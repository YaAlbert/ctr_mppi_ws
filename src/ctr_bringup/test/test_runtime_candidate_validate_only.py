import json, os, socket, stat, subprocess, sys
from dataclasses import replace
from hashlib import sha256
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import ctr_bringup.runtime_candidate_validate_only as candidate_module
import ctr_bringup.materialization_identity as materialization_module
from ctr_bringup.exercised_subject_identity import (
    AuthenticatedFile as SubjectAuthenticatedFile,
    canonical_exercised_subject_bytes,
    exercised_subject_identity,
    make_exercised_subject,
)
from ctr_bringup.materialization_identity import (
    LOGICAL_ALGORITHM_ID, MATERIALIZATION_PROJECTION_SCHEMA,
    PHYSICAL_REHASH_ALGORITHM_ID, PROJECTION_FRAMING_ALGORITHM_ID,
    MaterializationMember, MaterializationProjection,
    build_materialization_projection, canonical_materialization_projection_bytes,
    projection_identity_result, verify_materialization_root,
)
from ctr_bringup.runtime_candidate_validate_only import main, validate_frozen_candidate
from ctr_bringup.runtime_plan_validation import PROJECTION_SCHEMA_VERSION, RuntimeIssue, runtime_projection_identity


def _write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(data)


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode()


def _file_record(root, path):
    target = root / path
    data = target.read_bytes()
    return {'path': path, 'size': len(data), 'sha256': sha256(data).hexdigest()}


def _fixture(tmp_path):
    c = tmp_path / 'candidate'
    materialization_root = c / 'materialization'
    rr = materialization_root / 'runtime_root'
    rr.mkdir(parents=True)
    files = {
        'config/robot.yaml': b'robot: ctr\n',
        'launch/simulation.launch.py': b'def generate_launch_description(): return None\n',
        'pkg/main.py': b'def main(): return 0\n',
    }
    for index in range(169):
        files[f'pkg/generated_{index:03d}.py'] = f'VALUE = {index}\n'.encode()
    roles = {name: 'python_module' for name in files}
    roles['config/robot.yaml'] = 'configuration'; roles['launch/simulation.launch.py'] = 'launch_file'
    for name, data in files.items(): _write(rr / name, data)
    _write(materialization_root / 'material_only' / 'authority.txt', b'synthetic material authority\n')
    (materialization_root / 'empty').mkdir()
    for p in sorted(materialization_root.rglob('*'), key=lambda x: len(x.parts), reverse=True):
        p.chmod(0o555 if p.is_dir() else 0o444)
    materialization_root.chmod(0o555)
    members = [
        {'path': name, 'size_bytes': len(files[name]), 'sha256': sha256(files[name]).hexdigest(), 'mode': '0444', 'role': roles[name]}
        for name in sorted(files)
    ]
    projection = {'schema_version': PROJECTION_SCHEMA_VERSION, 'members': members}
    identity = runtime_projection_identity(projection)
    projection_path = 'manifests/payload_identity_projection.json'
    _write(c / projection_path, _canonical(projection))
    entrypoint = 'launch/simulation.launch.py'
    dependencies = [
        {'source': entrypoint, 'target': name, 'dependency_type': 'project', 'resolved': True}
        for name in sorted(files) if name != entrypoint
    ]
    dependencies += [
        {'source': entrypoint, 'target': name, 'dependency_type': 'external', 'resolved': True}
        for name in ('launch_ros', 'rclpy', 'yaml')
    ]
    graph = {'entrypoints': [entrypoint], 'project_nodes': sorted(files), 'dependencies': dependencies,
             'declared_external_dependencies': ['launch_ros', 'rclpy', 'yaml']}
    graph_path = 'manifests/runtime_dependency_graph.json'; _write(c / graph_path, _canonical(graph))

    plans = {}; plan_facts = {}
    for mode in ('production', 'offline', 'test_only'):
        raw = {'schema_version': 'ctr-runtime-plan-2', 'mode': mode,
               'production_runtime_identity': identity, 'runtime_root_role': 'AUTHENTICATED_RUNTIME_ROOT',
               'prospective_argv': ['ros2', entrypoint, '--domain', '232'],
               'project_owned_argv_indices': [1],
               'argv_bindings': [{'argv_index': 1, 'member_path': entrypoint}],
               'external_commands': [{'argv_index': 0, 'command': 'ros2', 'dependency': 'ros2'}],
               'argv_classifications': [
                   {'argv_index': 0, 'kind': 'external_command', 'value': 'ros2', 'dependency': 'ros2'},
                   {'argv_index': 1, 'kind': 'project_member', 'value': entrypoint, 'member_path': entrypoint},
                   {'argv_index': 2, 'kind': 'flag', 'value': '--domain'},
                   {'argv_index': 3, 'kind': 'integer', 'value': '232'}],
               'prospective_environment': {'ROS_DOMAIN_ID': '232'},
               'external_dependencies': ['launch_ros', 'ros2'],
               'policy': {'validate_only': True, 'allow_full_launch': False, 'launchable': False, 'execution_authorized': False}}
        data = _canonical(raw)
        for suffix in ('root', 'duplicate'):
            role = f'{mode}_{suffix}'; path = f'plans/{role}.json'
            _write(c / path, data); plans[role] = path
            plan_facts[role] = {'path': path, 'size': len(data), 'sha256': sha256(data).hexdigest()}
    plans_path = 'manifests/plans.json'
    _write(c / plans_path, _canonical({'plans': plans, 'allowed_external_dependencies': ['launch_ros', 'ros2']}))

    attachment_spec = {}
    for number in range(1, 9): attachment_spec[f'case-{number:03d}'] = (f'AUTH-{number:02d}', 'authorization', 'attachment_authorization')
    for number in range(75, 83): attachment_spec[f'case-{number:03d}'] = (f'ALLOC-{number-74:02d}', 'allocation', 'attachment_allocation')
    for number in range(83, 91): attachment_spec[f'case-{number:03d}'] = (f'RECEIPT-{number-82:02d}', 'durable_receipt', 'attachment_durable_receipt')
    for number in range(93, 97): attachment_spec[f'case-{number:03d}'] = (f'CHILD-{number-92:02d}', 'child_boundary', 'attachment_child_boundary')
    cats = [('authority', 43), ('output_isolation', 31), ('digest_allocation', 18), ('capsule', 11),
            ('regression', 1), ('relocation', 16), ('schema_negative', 16),
            ('additional_schema_negative', 8), ('report_consistency', 2), ('validate_only_capsule_policy', 4)]
    category_for = [category for category, count in cats for _ in range(count)]
    cases = []; attachments = []; package_identities = []
    for index, category in enumerate(category_for, 1):
        case_id = f'case-{index:03d}'; package_dir = c / 'focused_raw' / case_id; package_dir.mkdir(parents=True)
        raw_members = []
        for role in ('invocation_metadata', 'events', 'event_manifest', 'raw_capture_manifest', 'raw_result'):
            path = package_dir / f'{role}.json'; data = _canonical({'case_id': case_id, 'role': role}); _write(path, data)
            raw_members.append({'path': path.relative_to(c).as_posix(), 'role': role, 'size': len(data), 'sha256': sha256(data).hexdigest()})
        attachment_ids = []
        if case_id in attachment_spec:
            attachment_id, attachment_role, raw_role = attachment_spec[case_id]
            path = package_dir / 'attachment.bin'; data = f'attachment:{attachment_id}'.encode(); _write(path, data)
            record = {'attachment_id': attachment_id, 'role': attachment_role, 'case_id': case_id,
                      'path': path.relative_to(c).as_posix(), 'size': len(data), 'sha256': sha256(data).hexdigest()}
            attachments.append(record); attachment_ids = [attachment_id]
            raw_members.append({'path': record['path'], 'role': raw_role, 'size': record['size'], 'sha256': record['sha256']})
        package_projection = {'schema_version': 'ctr-focused-raw-package-projection-1', 'case_id': case_id,
                              'members': sorted(raw_members, key=lambda member: member['path'])}
        package_identity = sha256(_canonical(package_projection)).hexdigest(); package_identities.append((case_id, package_identity))
        raw_manifest = {'schema_version': 'ctr-focused-raw-package-1', 'case_id': case_id,
                        'members': raw_members, 'package_identity': package_identity}
        data = _canonical(raw_manifest); manifest_path = package_dir / 'manifest.json'; _write(manifest_path, data)
        origin = 'repository_production' if category in {'authority', 'digest_allocation', 'regression', 'schema_negative', 'additional_schema_negative'} else 'candidate_evidence_integrity'
        cases.append({'case_id': case_id, 'category': category, 'validator_origin': origin,
                      'validator_symbol': 'synthetic_validator', 'expected_code_or_result': 'PASS',
                      'observed_code_or_result': 'PASS', 'passed': True,
                      'raw_package': {'manifest_path': manifest_path.relative_to(c).as_posix(), 'manifest_size': len(data),
                                      'manifest_sha256': sha256(data).hexdigest(), 'package_identity': package_identity},
                      'attachment_ids': attachment_ids})
    focused_path = 'manifests/focused_results.json'; _write(c / focused_path, _canonical({'cases': cases, 'category_totals': dict(cats)}))
    attachments_path = 'manifests/attachments.json'
    _write(c / attachments_path, _canonical({'schema_version': 'ctr-focused-attachments-1', 'attachments': attachments}))

    capsule_policy = {'validate_only': True, 'allow_full_launch': False, 'launchable': False,
                      'execution_authorized': False, 'argv_role': 'validated_prospective_argv',
                      'domain_role': 'validated_prospective_environment', 'output_allocation_allowed': False,
                      'child_creation_allowed': False, 'output_allocation_performed': False}
    capsule_path = 'manifests/capsule.json'
    correction_path = 'reports/correction_report.md'; source_path = 'manifests/source_identity.json'
    predecessor_path = 'manifests/predecessor_preservation.json'
    materialization_path = 'manifests/materialization_projection.json'
    closure_projection_path = 'manifests/closure_source_projection.json'
    _write(c / correction_path, b'Authenticated correction report.\n')
    _write(c / predecessor_path, _canonical({'schema_version': 'synthetic-preservation-1', 'preserved': True}))

    materialization_projection = build_materialization_projection(materialization_root)
    materialization_raw = canonical_materialization_projection_bytes(materialization_projection)
    _write(c / materialization_path, materialization_raw)
    materialization_projection_result = projection_identity_result(materialization_projection)
    materialization_verification = verify_materialization_root(
        materialization_root, materialization_projection,
    )
    report_path = 'manifests/report_source.json'; closure_path = 'manifests/static_closure.json'
    bundle_path = 'manifests/validate_only_bundle.json'; inventory_path = 'manifests/candidate_inventory.json'
    authority_path = 'manifests/root_authority.json'
    exercised_subject_path = 'manifests/exercised_subject.json'
    bundle = {'schema': 'ctr-frozen-candidate-bundle-2', 'profile': 'slice-7f-final',
              'inventory_path': inventory_path, 'projection_path': projection_path,
              'runtime_root_path': 'materialization/runtime_root',
              'dependency_graph_path': graph_path, 'plans_manifest_path': plans_path,
              'focused_results_path': focused_path, 'attachment_manifest_path': attachments_path,
              'report_source_path': report_path, 'static_closure_path': closure_path,
              'capsule_path': capsule_path, 'correction_report_path': correction_path,
              'source_identity_path': source_path, 'predecessor_preservation_path': predecessor_path,
              'closure_source_projection_path': closure_projection_path,
              'materialization_projection_path': materialization_path,
              'materialization_root_path': 'materialization',
              'exercised_subject_path': exercised_subject_path}
    _write(c / bundle_path, _canonical(bundle))

    subject = make_exercised_subject(
        candidate_bundle=SubjectAuthenticatedFile(**_file_record(c, bundle_path)),
        runtime_projection=SubjectAuthenticatedFile(**_file_record(c, projection_path)),
        runtime_identity=identity,
        materialization_projection=SubjectAuthenticatedFile(
            **_file_record(c, materialization_path),
        ),
        materialization_logical_identity=(
            materialization_projection_result.logical_identity
        ),
    )
    subject_identity = exercised_subject_identity(subject)
    _write(c / exercised_subject_path, canonical_exercised_subject_bytes(subject))
    subject_binding = {
        'identity_algorithm_id': candidate_module.EXERCISED_SUBJECT_IDENTITY_ALGORITHM_ID,
        'logical_identity': subject_identity,
        **_file_record(c, exercised_subject_path),
    }
    repository_identity = candidate_module._repository_identity_snapshot()
    source_identity = {
        'schema_version': 'ctr-source-identity-2',
        'repository': repository_identity,
        'materialization': {
            'logical_algorithm_id': LOGICAL_ALGORITHM_ID,
            'logical_identity': materialization_projection_result.logical_identity,
            'materialization_authority': 'candidate-contained-descriptor-authenticated-root',
            'materialization_root_path': 'materialization',
            'physical_rehash': materialization_verification.physical_rehash,
            'physical_rehash_algorithm_id': PHYSICAL_REHASH_ALGORITHM_ID,
            'projection_framing_algorithm_id': PROJECTION_FRAMING_ALGORITHM_ID,
            'projection_framing_digest': materialization_projection_result.projection_framing_digest,
            'projection_path': materialization_path,
            'projection_schema': MATERIALIZATION_PROJECTION_SCHEMA,
            'projection_sha256': materialization_projection_result.projection_sha256,
            'projection_size': materialization_projection_result.projection_size,
            'runtime_binding_count': 172,
            'runtime_projection_identity': identity,
        },
        'exercised_subject': subject_binding,
        'historical_lineage': {
            'operative': False,
            'superseded_identities': [
                {'algorithm': algorithm, 'status': 'diagnostic_only', 'value': digest}
                for algorithm, digest in candidate_module._HISTORICAL_LINEAGE
            ],
        },
    }
    _write(c / source_path, _canonical(source_identity))
    _write(c / capsule_path, _canonical({
        **capsule_policy,
        'runtime_identity': identity,
        'exercised_subject_identity': subject_identity,
    }))

    exclusions = {inventory_path, authority_path, closure_projection_path, closure_path, report_path}
    for p in c.rglob('*'):
        if p.is_file(): p.chmod(0o444)
    records = []
    for p in sorted(c.rglob('*')):
        relative = p.relative_to(c).as_posix()
        if p.is_file() and relative not in exclusions:
            records.append({'path': relative, 'size': p.stat().st_size,
                            'sha256': sha256(p.read_bytes()).hexdigest(),
                            'mode': f'{stat.S_IMODE(p.stat().st_mode):04o}'})
    _write(c / inventory_path, _canonical({'schema': 'ctr-frozen-candidate-inventory-2', 'files': records}))
    (c / inventory_path).chmod(0o444)
    source_facts = tuple(candidate_module._closure_source_fact(
        candidate_module._FileFact(record['path'], record['size'], record['sha256'], record['mode']),
        materialization_root='materialization',
        runtime_root='materialization/runtime_root',
    ) for record in records)
    closure_projection = candidate_module._closure_projection_value(source_facts, exclusions)
    _write(c / closure_projection_path, _canonical(closure_projection)); (c / closure_projection_path).chmod(0o444)
    closure_projection_facts = candidate_module._ClosureProjectionFacts(
        candidate_module._FileFact(
            closure_projection_path, (c / closure_projection_path).stat().st_size,
            sha256((c / closure_projection_path).read_bytes()).hexdigest(), '0444',
        ),
        source_facts, tuple(sorted(exclusions)),
        tuple(sorted(closure_projection['category_counts'].items())),
    )

    raw_aggregate = sha256(_canonical({'schema_version': 'ctr-raw-package-aggregate-1',
                                       'packages': [{'case_id': case_id, 'package_identity': digest}
                                                    for case_id, digest in package_identities]})).hexdigest()
    role_categories = {'authorization': 'authorization_attachments', 'allocation': 'allocation_attachments',
                       'durable_receipt': 'durable_receipts', 'child_boundary': 'child_boundary'}
    observation_items = []
    def observe(key, category, value, source_paths=()):
        observation_items.append((key, category, value, sorted(set(source_paths))))
    observe('candidate.inventory', 'candidate_inventory', {'path': inventory_path, 'member_count': len(records)}, [inventory_path])
    observe('closure_source_projection.coverage', 'closure_source_projection', {
            'path': closure_projection_path, 'size': closure_projection_facts.file.size,
            'sha256': closure_projection_facts.file.sha256, 'fact_count': len(source_facts),
            'category_count': len(closure_projection['category_counts'])}, [closure_projection_path])
    observe('runtime.projection', 'runtime_projection', {'path': projection_path, 'identity': identity}, [projection_path])
    observe('runtime.physical', 'runtime_physical', {
            'root': 'materialization/runtime_root', 'member_count': 172, 'declared_count': 172,
            'physical_regular_file_count': 172, 'matched_count': 172,
            'issue_count': 0, 'reconciled': True},
            [f'materialization/runtime_root/{name}' for name in sorted(files)])
    observe('runtime.dependencies', 'runtime_dependencies', {
            'path': graph_path, 'edge_count': 174, 'reachable_member_count': 172,
            'external_dependency_count': 3, 'unresolved': 0}, [graph_path])
    for role in ('production_root', 'production_duplicate', 'offline_root', 'offline_duplicate', 'test_only_root', 'test_only_duplicate'):
        fact = plan_facts[role]
        observe(f'plan.{role}', f'plan_{role}', {'role': role, **fact,
                'embedded_runtime_identity': identity, 'canonical_runtime_identity': identity}, [fact['path']])
    for category, count in cats:
        observe(f'focused.{category}', f'focused_{category}', count, [focused_path])
    observe('raw_packages.summary', 'raw_packages', {'count': 150, 'aggregate_sha256': raw_aggregate},
            [case['raw_package']['manifest_path'] for case in cases])
    for role in ('authorization', 'allocation', 'durable_receipt', 'child_boundary'):
        selected = sorted((record for record in attachments if record['role'] == role), key=lambda record: record['attachment_id'])
        projection_value = {'schema_version': 'ctr-attachment-role-aggregate-1', 'role': role,
                            'attachments': [{key: record[key] for key in ('attachment_id', 'case_id', 'path', 'size', 'sha256')} for record in selected]}
        key_role = 'durable_receipts' if role == 'durable_receipt' else role
        observe(f'attachments.{key_role}', role_categories[role],
                {'count': len(selected), 'aggregate_sha256': sha256(_canonical(projection_value)).hexdigest()},
                [record['path'] for record in selected])
    observe('materialization.projection', 'materialization_projection', {
            'path': materialization_path, 'size': materialization_projection_result.projection_size,
            'sha256': materialization_projection_result.projection_sha256,
            'logical_identity': materialization_projection_result.logical_identity,
            'materialization_root_path': 'materialization',
            'projection_framing_digest': materialization_projection_result.projection_framing_digest,
            'physical_rehash': materialization_verification.physical_rehash,
            'physically_observed_member_count': len(materialization_verification.observed_members)},
            [materialization_path] + [
                'materialization/' + member.path
                for member in materialization_projection.members
                if member.kind == 'regular_file'
            ])
    observe('exercised_subject.identity', 'exercised_subject', {
            **subject_binding,
            'candidate_bundle': subject.candidate_bundle.as_dict(),
            'runtime_projection': subject.runtime_projection.as_dict(),
            'runtime_identity': subject.runtime_identity,
            'materialization_projection': subject.materialization_projection.as_dict(),
            'materialization_logical_identity': subject.materialization_logical_identity},
            [exercised_subject_path])
    observe('correction_report.bytes', 'correction_report', _file_record(c, correction_path), [correction_path])
    observe('capsule.policy', 'capsule_policy', {**_file_record(c, capsule_path),
            'policy': capsule_policy, 'exercised_subject_identity': subject_identity}, [capsule_path])
    observe('candidate.basename', 'candidate_path', c.name)
    observe('source.identity', 'source_identity', {**_file_record(c, source_path), 'authentication_scope': 'root_authority_inventory'}, [source_path])
    observe('predecessor.preservation', 'predecessor_preservation', {**_file_record(c, predecessor_path),
            'authentication_scope': 'root_authority_inventory', 'external_predecessors_inspected': False}, [predecessor_path])
    side_effects = {'output_allocation_performed': False, 'process_factory_calls': 0, 'real_popen_calls': 0,
                    'target_children': 0, 'rclpy_activity': 0, 'ros_commands': 0, 'dds_participants': 0}
    observe('side_effect.boundary', 'side_effect_boundary', side_effects)
    observe('materialization.runtime_binding', 'materialization_runtime_binding',
            {'runtime_identity': identity, 'bound_members': 172}, [materialization_path, projection_path])
    observations = candidate_module._make_observations(observation_items)
    assert len(observations) == 35
    closure = candidate_module._expected_static_closure_value(observations, closure_projection_facts)
    _write(c / closure_path, _canonical(closure)); (c / closure_path).chmod(0o444)

    report = {'schema_version': 'ctr-report-source-2',
              'runtime': {'identity': identity, 'member_count': 172, 'dependency_edge_count': 174},
              'plans': plan_facts,
              'focused': {'case_count': 150, 'raw_package_count': 150, 'categories': dict(cats),
                          'validator_origins': {'repository_production': 86, 'candidate_evidence_integrity': 64}},
              'attachments': {'attachment_count': 28,
                              'roles': {'authorization': 8, 'allocation': 8, 'durable_receipt': 8, 'child_boundary': 4}},
              'closure_source_projection': {'path': closure_projection_path,
                  'size': closure_projection_facts.file.size, 'sha256': closure_projection_facts.file.sha256,
                  'fact_count': len(source_facts), 'category_count': len(closure_projection['category_counts'])},
              'static_closure': {key: closure[key] for key in (
                  'critical_count', 'authenticated_source_count', 'check_count', 'category_count', 'failed_checks')},
              'materialization': {'projection_path': materialization_path,
                  'projection_schema': MATERIALIZATION_PROJECTION_SCHEMA,
                  'projection_size': materialization_projection_result.projection_size,
                  'projection_sha256': materialization_projection_result.projection_sha256,
                  'logical_identity': materialization_projection_result.logical_identity,
                  'materialization_root_path': 'materialization',
                  'projection_framing_digest': materialization_projection_result.projection_framing_digest,
                  'physical_rehash': materialization_verification.physical_rehash,
                  'runtime_binding_count': 172},
              'exercised_subject': subject_binding,
              'correction_report': _file_record(c, correction_path),
              'capsule': {**_file_record(c, capsule_path), 'policy': capsule_policy,
                          'exercised_subject_identity': subject_identity},
              'candidate_basename': c.name}
    _write(c / report_path, _canonical(report)); (c / report_path).chmod(0o444)

    authority_bindings = [
        ('candidate_bundle', bundle_path), ('candidate_inventory', inventory_path),
        ('runtime_projection', projection_path), ('runtime_dependency_graph', graph_path),
        ('six_plan_manifest', plans_path), ('focused_results', focused_path),
        ('attachment_manifest', attachments_path),
        ('closure_source_projection', closure_projection_path),
        ('materialization_projection', materialization_path),
        ('exercised_subject', exercised_subject_path), ('source_identity', source_path),
        ('report_source', report_path), ('static_closure', closure_path), ('capsule', capsule_path),
    ]
    children = [{**_file_record(c, path), 'role': role} for role, path in authority_bindings]
    _write(c / authority_path, _canonical({'schema': 'ctr-root-authority-2', 'children': children}))
    authority = sha256((c / authority_path).read_bytes()).hexdigest()
    for p in c.rglob('*'):
        if p.is_dir(): p.chmod(0o555)
        elif p.is_file(): p.chmod(0o444)
    c.chmod(0o555)
    return c, authority, identity


def _reseal(candidate, *, normalize_modes=True):
    inventory_path = 'manifests/candidate_inventory.json'
    authority_path = 'manifests/root_authority.json'
    bundle = json.loads((candidate / 'manifests/validate_only_bundle.json').read_text())
    exclusions = {
        inventory_path, authority_path, bundle['closure_source_projection_path'],
        bundle['static_closure_path'], bundle['report_source_path'],
    }
    if normalize_modes:
        for path in candidate.rglob('*'):
            if path.is_file(): path.chmod(0o444)
    inventory = candidate / inventory_path; inventory.chmod(0o644)
    records = []
    for path in sorted(candidate.rglob('*')):
        relative = path.relative_to(candidate).as_posix()
        if path.is_file() and relative not in exclusions:
            data = path.read_bytes()
            records.append({'path': relative, 'size': len(data), 'sha256': sha256(data).hexdigest(),
                            'mode': f'{stat.S_IMODE(path.stat().st_mode):04o}'})
    inventory.write_bytes(_canonical({'schema': 'ctr-frozen-candidate-inventory-2', 'files': records}))
    inventory.chmod(0o444)
    bindings = [
        ('candidate_bundle', 'manifests/validate_only_bundle.json'),
        ('candidate_inventory', inventory_path),
        ('runtime_projection', bundle['projection_path']),
        ('runtime_dependency_graph', bundle['dependency_graph_path']),
        ('six_plan_manifest', bundle['plans_manifest_path']),
        ('focused_results', bundle['focused_results_path']),
        ('attachment_manifest', bundle['attachment_manifest_path']),
        ('closure_source_projection', bundle['closure_source_projection_path']),
        ('materialization_projection', bundle['materialization_projection_path']),
        ('exercised_subject', bundle['exercised_subject_path']),
        ('source_identity', bundle['source_identity_path']),
        ('report_source', bundle['report_source_path']),
        ('static_closure', bundle['static_closure_path']),
        ('capsule', bundle['capsule_path']),
    ]
    authority_file = candidate / authority_path; authority_file.chmod(0o644)
    children = [{**_file_record(candidate, path), 'role': role} for role, path in bindings]
    authority_file.write_bytes(_canonical({'schema': 'ctr-root-authority-2', 'children': children}))
    authority_file.chmod(0o444)
    for path in candidate.rglob('*'):
        if path.is_dir():
            path.chmod(0o555)
        elif normalize_modes:
            path.chmod(0o444)
    candidate.chmod(0o555)
    return sha256(authority_file.read_bytes()).hexdigest()


def _reseal_authority_only(candidate):
    bundle = json.loads((candidate / 'manifests/validate_only_bundle.json').read_text())
    bindings = [
        ('candidate_bundle', 'manifests/validate_only_bundle.json'),
        ('candidate_inventory', bundle['inventory_path']),
        ('runtime_projection', bundle['projection_path']),
        ('runtime_dependency_graph', bundle['dependency_graph_path']),
        ('six_plan_manifest', bundle['plans_manifest_path']),
        ('focused_results', bundle['focused_results_path']),
        ('attachment_manifest', bundle['attachment_manifest_path']),
        ('closure_source_projection', bundle['closure_source_projection_path']),
        ('materialization_projection', bundle['materialization_projection_path']),
        ('exercised_subject', bundle['exercised_subject_path']),
        ('source_identity', bundle['source_identity_path']),
        ('report_source', bundle['report_source_path']),
        ('static_closure', bundle['static_closure_path']),
        ('capsule', bundle['capsule_path']),
    ]
    authority_file = candidate / 'manifests/root_authority.json'; authority_file.chmod(0o644)
    children = [{**_file_record(candidate, path), 'role': role} for role, path in bindings]
    authority_file.write_bytes(_canonical({'schema': 'ctr-root-authority-2', 'children': children}))
    authority_file.chmod(0o444)
    return sha256(authority_file.read_bytes()).hexdigest()


def _coherently_reseal_projection_claim(candidate, projection_value, identity):
    """Reseal every downstream claim without creating the projected members."""

    bundle = json.loads((candidate / 'manifests/validate_only_bundle.json').read_text())
    projection_path = candidate / bundle['materialization_projection_path']
    projection_value = dict(projection_value)
    projection_value['members'] = sorted(
        projection_value['members'], key=lambda member: member['path'].encode('utf-8'),
    )
    projection_raw = _canonical(projection_value)
    framing = sha256()
    framing.update(materialization_module._PHYSICAL_DOMAIN)
    for record in projection_value['members']:
        encoded = _canonical(record)
        framing.update(len(encoded).to_bytes(8, 'big'))
        framing.update(encoded)
    claimed = {
        'projection_size': len(projection_raw),
        'projection_sha256': sha256(projection_raw).hexdigest(),
        'logical_identity': sha256(
            materialization_module._LOGICAL_DOMAIN + projection_raw,
        ).hexdigest(),
        'projection_framing_digest': framing.hexdigest(),
        'physical_rehash': framing.hexdigest(),
        'observed_member_count': len(projection_value['members']),
    }
    projection_path.chmod(0o644)
    projection_path.write_bytes(projection_raw)
    projection_path.chmod(0o444)

    subject = make_exercised_subject(
        candidate_bundle=SubjectAuthenticatedFile(**_file_record(
            candidate, 'manifests/validate_only_bundle.json',
        )),
        runtime_projection=SubjectAuthenticatedFile(**_file_record(
            candidate, bundle['projection_path'],
        )),
        runtime_identity=identity,
        materialization_projection=SubjectAuthenticatedFile(**_file_record(
            candidate, bundle['materialization_projection_path'],
        )),
        materialization_logical_identity=claimed['logical_identity'],
    )
    subject_identity = exercised_subject_identity(subject)
    subject_path = candidate / bundle['exercised_subject_path']
    subject_path.chmod(0o644)
    subject_path.write_bytes(canonical_exercised_subject_bytes(subject))
    subject_path.chmod(0o444)
    subject_binding = {
        'identity_algorithm_id': candidate_module.EXERCISED_SUBJECT_IDENTITY_ALGORITHM_ID,
        'logical_identity': subject_identity,
        **_file_record(candidate, bundle['exercised_subject_path']),
    }

    source_path = candidate / bundle['source_identity_path']
    source_path.chmod(0o644)
    source = json.loads(source_path.read_text())
    source['materialization'].update({
        'logical_identity': claimed['logical_identity'],
        'physical_rehash': claimed['physical_rehash'],
        'projection_framing_digest': claimed['projection_framing_digest'],
        'projection_sha256': claimed['projection_sha256'],
        'projection_size': claimed['projection_size'],
    })
    source['exercised_subject'] = subject_binding
    source_path.write_bytes(_canonical(source)); source_path.chmod(0o444)

    report_path = candidate / bundle['report_source_path']
    report_path.chmod(0o644)
    report = json.loads(report_path.read_text())
    report['materialization'].update({
        'logical_identity': claimed['logical_identity'],
        'physical_rehash': claimed['physical_rehash'],
        'projection_framing_digest': claimed['projection_framing_digest'],
        'projection_sha256': claimed['projection_sha256'],
        'projection_size': claimed['projection_size'],
    })
    report['exercised_subject'] = subject_binding
    report_path.write_bytes(_canonical(report)); report_path.chmod(0o444)

    capsule_path = candidate / bundle['capsule_path']
    capsule_path.chmod(0o644)
    capsule = json.loads(capsule_path.read_text())
    capsule['exercised_subject_identity'] = subject_identity
    capsule_path.write_bytes(_canonical(capsule)); capsule_path.chmod(0o444)
    report['capsule'] = {
        **_file_record(candidate, bundle['capsule_path']),
        'policy': report['capsule']['policy'],
        'exercised_subject_identity': subject_identity,
    }
    report_path.chmod(0o644)
    report_path.write_bytes(_canonical(report)); report_path.chmod(0o444)

    authority = _reseal(candidate)
    inventory = json.loads((candidate / bundle['inventory_path']).read_text())
    exclusions = {
        bundle['inventory_path'], 'manifests/root_authority.json',
        bundle['closure_source_projection_path'], bundle['static_closure_path'],
        bundle['report_source_path'],
    }
    source_facts = tuple(candidate_module._closure_source_fact(
        candidate_module._FileFact(
            record['path'], record['size'], record['sha256'], record['mode'],
        ),
        materialization_root=bundle['materialization_root_path'],
        runtime_root=bundle['runtime_root_path'],
    ) for record in inventory['files'])
    closure_projection_value = candidate_module._closure_projection_value(
        source_facts, exclusions,
    )
    closure_projection_path = candidate / bundle['closure_source_projection_path']
    closure_projection_path.chmod(0o644)
    closure_projection_path.write_bytes(_canonical(closure_projection_value))
    closure_projection_path.chmod(0o444)
    closure_projection_file = candidate_module._FileFact(
        bundle['closure_source_projection_path'],
        closure_projection_path.stat().st_size,
        sha256(closure_projection_path.read_bytes()).hexdigest(), '0444',
    )
    closure_projection_facts = candidate_module._ClosureProjectionFacts(
        closure_projection_file, source_facts, tuple(sorted(exclusions)),
        tuple(sorted(closure_projection_value['category_counts'].items())),
    )

    closure_path = candidate / bundle['static_closure_path']
    old_closure = json.loads(closure_path.read_text())
    critical = {
        check['observation_key']: check
        for check in old_closure['checks']
        if check['kind'] == 'critical_observation'
    }
    projection_source_paths = [bundle['materialization_projection_path']] + [
        bundle['materialization_root_path'].rstrip('/') + '/' + record['path']
        for record in projection_value['members']
        if record['kind'] == 'regular_file'
    ]
    critical['materialization.projection']['expected'] = {
        'path': bundle['materialization_projection_path'],
        'size': claimed['projection_size'],
        'sha256': claimed['projection_sha256'],
        'logical_identity': claimed['logical_identity'],
        'materialization_root_path': bundle['materialization_root_path'],
        'projection_framing_digest': claimed['projection_framing_digest'],
        'physical_rehash': claimed['physical_rehash'],
        'physically_observed_member_count': claimed['observed_member_count'],
    }
    critical['materialization.projection']['observed'] = dict(
        critical['materialization.projection']['expected'],
    )
    critical['materialization.projection']['source_paths'] = sorted(
        set(projection_source_paths),
    )
    critical['exercised_subject.identity']['expected'] = {
        **subject_binding,
        'candidate_bundle': subject.candidate_bundle.as_dict(),
        'runtime_projection': subject.runtime_projection.as_dict(),
        'runtime_identity': subject.runtime_identity,
        'materialization_projection': subject.materialization_projection.as_dict(),
        'materialization_logical_identity': subject.materialization_logical_identity,
    }
    critical['exercised_subject.identity']['observed'] = dict(
        critical['exercised_subject.identity']['expected'],
    )
    critical['capsule.policy']['expected'] = {
        **_file_record(candidate, bundle['capsule_path']),
        'policy': report['capsule']['policy'],
        'exercised_subject_identity': subject_identity,
    }
    critical['capsule.policy']['observed'] = dict(
        critical['capsule.policy']['expected'],
    )
    source_record = _file_record(candidate, bundle['source_identity_path'])
    source_record['authentication_scope'] = 'root_authority_inventory'
    critical['source.identity']['expected'] = source_record
    critical['source.identity']['observed'] = dict(source_record)
    coverage = {
        'path': bundle['closure_source_projection_path'],
        'size': closure_projection_file.size,
        'sha256': closure_projection_file.sha256,
        'fact_count': len(source_facts),
        'category_count': len(closure_projection_facts.category_totals),
    }
    critical['closure_source_projection.coverage']['expected'] = coverage
    critical['closure_source_projection.coverage']['observed'] = dict(coverage)
    observations = candidate_module._make_observations([
        (
            key, record['category'], record['expected'],
            record['source_paths'],
        )
        for key, record in critical.items()
    ])
    closure = candidate_module._expected_static_closure_value(
        observations, closure_projection_facts,
    )
    closure_path.chmod(0o644); closure_path.write_bytes(_canonical(closure)); closure_path.chmod(0o444)

    report_path.chmod(0o644)
    report = json.loads(report_path.read_text())
    report['closure_source_projection'] = coverage
    report['static_closure'] = {key: closure[key] for key in (
        'critical_count', 'authenticated_source_count', 'check_count',
        'category_count', 'failed_checks',
    )}
    report_path.write_bytes(_canonical(report)); report_path.chmod(0o444)
    authority = _reseal(candidate)
    return authority, claimed


def _mutate_json(candidate, relative, mutate):
    path = candidate / relative; path.chmod(0o644)
    value = json.loads(path.read_text()); mutate(value)
    path.write_bytes(_canonical(value)); path.chmod(0o444)


def _rewrite_raw_package(candidate, case_id, mutate, *, recompute_identity=True):
    manifest_relative = f'focused_raw/{case_id}/manifest.json'
    manifest_path = candidate / manifest_relative
    manifest_path.chmod(0o644)
    raw = json.loads(manifest_path.read_text())
    mutate(raw)
    if recompute_identity:
        projection = {
            'schema_version': 'ctr-focused-raw-package-projection-1',
            'case_id': raw['case_id'],
            'members': sorted(raw['members'], key=lambda member: member['path']),
        }
        raw['package_identity'] = sha256(_canonical(projection)).hexdigest()
    raw_bytes = _canonical(raw)
    manifest_path.write_bytes(raw_bytes); manifest_path.chmod(0o444)
    focused_path = candidate / 'manifests/focused_results.json'; focused_path.chmod(0o644)
    focused = json.loads(focused_path.read_text())
    case = next(record for record in focused['cases'] if record['case_id'] == case_id)
    case['raw_package'].update({
        'manifest_size': len(raw_bytes),
        'manifest_sha256': sha256(raw_bytes).hexdigest(),
        'package_identity': raw['package_identity'],
    })
    focused_path.write_bytes(_canonical(focused)); focused_path.chmod(0o444)
    return raw


def _assert_single_failure(result, stage, code=None):
    assert result.overall == 'FAIL'
    failures = [trace for trace in result.traces if trace.status == 'FAIL']
    assert len(failures) == 1
    assert failures[0].name == stage
    assert result.traces[-1] is failures[0]
    prefix = result.traces[:-1]
    assert all(trace.status == 'PASS' for trace in prefix)
    assert tuple(trace.name for trace in prefix) == candidate_module.TRACE_NAMES[:len(prefix)]
    assert candidate_module.TRACE_NAMES.index(stage) >= len(prefix)
    if code is not None:
        assert failures[0].code == code


def _refresh_closure_observations(candidate, authority, identity):
    bundle = json.loads((candidate / 'manifests/validate_only_bundle.json').read_text())
    inventory = json.loads((candidate / bundle['inventory_path']).read_text())
    exclusions = {
        bundle['inventory_path'], 'manifests/root_authority.json',
        bundle['closure_source_projection_path'], bundle['static_closure_path'],
        bundle['report_source_path'],
    }
    source_facts = tuple(candidate_module._closure_source_fact(
        candidate_module._FileFact(record['path'], record['size'], record['sha256'], record['mode']),
        materialization_root=bundle['materialization_root_path'],
        runtime_root=bundle['runtime_root_path'],
    ) for record in inventory['files'])
    projection_value = candidate_module._closure_projection_value(source_facts, exclusions)
    projection_path = candidate / bundle['closure_source_projection_path']
    projection_path.chmod(0o644); projection_path.write_bytes(_canonical(projection_value)); projection_path.chmod(0o444)
    authority = _reseal(candidate)
    facts = _upstream_facts(candidate, authority, identity)
    observations = candidate_module._build_required_observations(facts)
    closure = candidate_module._expected_static_closure_value(observations, facts.closure_source)
    closure_path = candidate / bundle['static_closure_path']; closure_path.chmod(0o644)
    closure_path.write_bytes(_canonical(closure)); closure_path.chmod(0o444)
    report_path = candidate / bundle['report_source_path']; report_path.chmod(0o644)
    report = json.loads(report_path.read_text())
    report['closure_source_projection'] = {
        'path': facts.closure_source.file.path, 'size': facts.closure_source.file.size,
        'sha256': facts.closure_source.file.sha256,
        'fact_count': len(facts.closure_source.records),
        'category_count': len(facts.closure_source.category_totals),
    }
    report['static_closure'] = {key: closure[key] for key in (
        'critical_count', 'authenticated_source_count', 'check_count',
        'category_count', 'failed_checks')}
    report_path.write_bytes(_canonical(report)); report_path.chmod(0o444)
    return _reseal(candidate)


def _upstream_facts(candidate, authority, identity):
    with candidate_module._CandidateReadSession(candidate) as session:
        bundle = candidate_module._load_bundle(session, 'manifests/validate_only_bundle.json')
        inventory = candidate_module._validate_inventory(session, bundle)
        authority_facts = candidate_module._validate_authority(session, bundle, inventory, authority)
        runtime = candidate_module._validate_runtime(session, bundle, inventory, authority_facts, identity)
        materialization = candidate_module._validate_materialization_contract(
            session, bundle, inventory, runtime,
        )
        exercised_subject = candidate_module._validate_exercised_subject_contract(
            session, bundle, inventory, authority_facts, runtime, materialization,
        )
        plans = candidate_module._validate_plans(session, bundle, runtime, identity)
        focused = candidate_module._validate_focused(session, bundle, inventory)
        attachments = candidate_module._validate_attachments(session, bundle, inventory, focused)
        capsule = candidate_module._validate_capsule(
            session, bundle, inventory, runtime, exercised_subject,
        )
        report_inputs = candidate_module._resolve_report_inputs(session, bundle, inventory)
        closure_source = candidate_module._validate_closure_source_projection(
            session, bundle, inventory,
        )
        invocation = candidate_module._InvocationFacts(candidate.name, authority, identity,
                                                        tuple(candidate_module._SIDE_EFFECTS.items()))
        return candidate_module._CandidateFacts(
            bundle, inventory, authority_facts, runtime, plans, focused,
            attachments, capsule, report_inputs, closure_source,
            materialization, exercised_subject, invocation,
        )


def test_positive_candidate_has_ten_passing_traces(tmp_path):
    c,a,i=_fixture(tmp_path); r=validate_frozen_candidate(c,expected_root_authority=a,expected_runtime_identity=i); assert r.overall=='PASS'; assert len(r.traces)==10


def test_positive_subject_candidate_has_zero_dynamic_side_effects(
    tmp_path, monkeypatch,
):
    candidate, authority, identity = _fixture(tmp_path)
    def observation():
        records = []
        for path in [candidate, *sorted(candidate.rglob('*'))]:
            status = path.lstat()
            relative = '.' if path == candidate else path.relative_to(candidate).as_posix()
            digest = sha256(path.read_bytes()).hexdigest() if path.is_file() else None
            records.append((relative, status.st_dev, status.st_ino, status.st_mode,
                            status.st_nlink, status.st_size, status.st_mtime_ns, digest))
        return tuple(records)
    before_tree = observation()
    before_environment = dict(os.environ)
    before_fds = len(os.listdir('/proc/self/fd'))
    before_ros_modules = {name for name in sys.modules
                          if name == 'rclpy' or name.startswith(('rclpy.', 'launch.', 'launch_ros.'))}
    original_open = candidate_module.os.open
    def read_only_open(path, flags, *args, **kwargs):
        forbidden = os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_TRUNC | os.O_APPEND
        assert flags & forbidden == 0
        return original_open(path, flags, *args, **kwargs)
    def forbidden(*_args, **_kwargs):
        raise AssertionError('forbidden validate-only side effect')
    monkeypatch.setattr(candidate_module.os, 'open', read_only_open)
    monkeypatch.setattr(
        candidate_module.os, 'supports_dir_fd',
        set(candidate_module.os.supports_dir_fd) | {read_only_open},
    )
    for name in ('write_bytes', 'write_text', 'chmod', 'mkdir', 'unlink', 'rename', 'replace'):
        monkeypatch.setattr(Path, name, forbidden)
    monkeypatch.setattr(subprocess, 'Popen', forbidden)
    monkeypatch.setattr(socket, 'socket', forbidden)
    result = validate_frozen_candidate(
        candidate, expected_root_authority=authority,
        expected_runtime_identity=identity,
    )
    assert result.overall == 'PASS', result.traces[-1]
    assert dict(result.side_effects) == candidate_module._SIDE_EFFECTS
    assert observation() == before_tree
    assert dict(os.environ) == before_environment
    assert len(os.listdir('/proc/self/fd')) == before_fds
    after_ros_modules = {name for name in sys.modules
                         if name == 'rclpy' or name.startswith(('rclpy.', 'launch.', 'launch_ros.'))}
    assert after_ros_modules == before_ros_modules


@pytest.mark.parametrize(
    ('mutation', 'code'),
    [
        ('unknown_field', 'SUBJECT_FIELDS'),
        ('missing_field', 'SUBJECT_FIELDS'),
        ('wrong_schema', 'SUBJECT_SCHEMA'),
        ('invalid_digest', 'SUBJECT_DIGEST'),
        ('invalid_size', 'SUBJECT_SIZE'),
        ('invalid_type', 'SUBJECT_TYPE'),
        ('unsafe_path', 'SUBJECT_PATH'),
        ('bundle_binding', 'SUBJECT_BUNDLE_BINDING_MISMATCH'),
        ('runtime_identity', 'SUBJECT_RUNTIME_IDENTITY_MISMATCH'),
        (
            'materialization_identity',
            'SUBJECT_MATERIALIZATION_LOGICAL_IDENTITY_MISMATCH',
        ),
        ('stale_v2_identity', 'SUBJECT_STALE_DIAGNOSTIC_IDENTITY'),
    ],
)
def test_exercised_subject_mutations_reach_subject_stage(tmp_path, mutation, code):
    candidate, _, identity = _fixture(tmp_path)
    def alter(value):
        if mutation == 'unknown_field': value['unknown'] = 1
        elif mutation == 'missing_field': value.pop('runtime_identity')
        elif mutation == 'wrong_schema': value['schema_version'] = 'ctr-exercised-subject-1'
        elif mutation == 'invalid_digest': value['runtime_identity'] = 'x' * 64
        elif mutation == 'invalid_size': value['candidate_bundle']['size'] = 0
        elif mutation == 'invalid_type': value['runtime_identity'] = 7
        elif mutation == 'unsafe_path': value['candidate_bundle']['path'] = '../bundle.json'
        elif mutation == 'bundle_binding': value['candidate_bundle']['size'] += 1
        elif mutation == 'runtime_identity': value['runtime_identity'] = 'f' * 64
        elif mutation == 'materialization_identity':
            value['materialization_logical_identity'] = 'f' * 64
        else:
            value['runtime_identity'] = (
                candidate_module.STALE_V2_DIAGNOSTIC_IDENTITY
            )
    _mutate_json(candidate, 'manifests/exercised_subject.json', alter)
    authority = _reseal(candidate)
    result = validate_frozen_candidate(
        candidate, expected_root_authority=authority,
        expected_runtime_identity=identity,
    )
    _assert_single_failure(result, 'runtime_physical', code)


def test_noncanonical_exercised_subject_bytes_reach_subject_stage(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    path = candidate / 'manifests/exercised_subject.json'
    path.chmod(0o644)
    value = json.loads(path.read_text())
    path.write_bytes(json.dumps(value, indent=2).encode() + b'\n')
    path.chmod(0o444)
    authority = _reseal(candidate)
    result = validate_frozen_candidate(
        candidate, expected_root_authority=authority,
        expected_runtime_identity=identity,
    )
    _assert_single_failure(result, 'runtime_physical', 'SUBJECT_NONCANONICAL')


@pytest.mark.parametrize(
    ('mutation', 'code'),
    [
        ('missing', 'SUBJECT_AUTHORITY_ROLE_MISSING'),
        ('duplicate', 'SUBJECT_AUTHORITY_ROLE_DUPLICATE'),
        ('physical_digest', 'SUBJECT_PHYSICAL_DIGEST_MISMATCH'),
        ('legacy_13', 'LEGACY_13_AUTHORITY_ROLES'),
    ],
)
def test_subject_authority_role_failures_are_stable(tmp_path, mutation, code):
    candidate, _, identity = _fixture(tmp_path)
    authority_path = candidate / 'manifests/root_authority.json'
    authority_path.chmod(0o644)
    value = json.loads(authority_path.read_text())
    children = value['children']
    subject = next(child for child in children if child['role'] == 'exercised_subject')
    if mutation == 'missing': subject['role'] = 'unknown_subject_role'
    elif mutation == 'duplicate':
        next(child for child in children if child['role'] == 'source_identity')['role'] = 'exercised_subject'
    elif mutation == 'physical_digest': subject['sha256'] = '0' * 64
    else: children.remove(subject)
    authority_path.write_bytes(_canonical(value)); authority_path.chmod(0o444)
    authority = sha256(authority_path.read_bytes()).hexdigest()
    result = validate_frozen_candidate(
        candidate, expected_root_authority=authority,
        expected_runtime_identity=identity,
    )
    _assert_single_failure(result, 'root_authority', code)


def test_subject_record_must_be_present_in_inventory(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    _mutate_json(
        candidate, 'manifests/candidate_inventory.json',
        lambda value: value.__setitem__(
            'files', [record for record in value['files']
                      if record['path'] != 'manifests/exercised_subject.json'],
        ),
    )
    authority = _reseal_authority_only(candidate)
    result = validate_frozen_candidate(
        candidate, expected_root_authority=authority,
        expected_runtime_identity=identity,
    )
    _assert_single_failure(
        result, 'candidate_inventory', 'SUBJECT_RECORD_NOT_IN_INVENTORY',
    )


@pytest.mark.parametrize(
    ('target', 'stage', 'code'),
    [
        ('source', 'runtime_physical', 'SOURCE_IDENTITY_SUBJECT_MISMATCH'),
        ('capsule', 'capsule_policy', 'CAPSULE_SUBJECT_MISMATCH'),
        ('report', 'report_and_static_closure', 'REPORT_SOURCE_SUBJECT_MISMATCH'),
        ('closure', 'report_and_static_closure', 'CLOSURE_SUBJECT_MISMATCH'),
        ('contradictory', 'runtime_physical', 'SOURCE_IDENTITY_SUBJECT_MISMATCH'),
    ],
)
def test_downstream_subject_bindings_cannot_supply_identity(
    tmp_path, target, stage, code,
):
    candidate, _, identity = _fixture(tmp_path)
    if target in {'source', 'contradictory'}:
        _mutate_json(
            candidate, 'manifests/source_identity.json',
            lambda value: value['exercised_subject'].__setitem__(
                'logical_identity', 'f' * 64,
            ),
        )
    if target in {'capsule', 'contradictory'}:
        _mutate_json(
            candidate, 'manifests/capsule.json',
            lambda value: value.__setitem__(
                'exercised_subject_identity', 'f' * 64,
            ),
        )
    elif target == 'report':
        _mutate_json(
            candidate, 'manifests/report_source.json',
            lambda value: value['exercised_subject'].__setitem__(
                'logical_identity', 'f' * 64,
            ),
        )
    elif target == 'closure':
        def alter_closure(value):
            record = next(
                check for check in value['checks']
                if check.get('observation_key') == 'exercised_subject.identity'
            )
            record['expected']['logical_identity'] = 'f' * 64
        _mutate_json(candidate, 'manifests/static_closure.json', alter_closure)
    authority = _reseal(candidate)
    result = validate_frozen_candidate(
        candidate, expected_root_authority=authority,
        expected_runtime_identity=identity,
    )
    _assert_single_failure(result, stage, code)


def test_main_canonical_output_and_failure_code(tmp_path,capsys):
    c,a,i=_fixture(tmp_path); assert main(['--candidate-root',str(c),'--expected-root-authority',a,'--expected-runtime-identity',i])==0; assert json.loads(capsys.readouterr().out)['overall']=='PASS'
    assert main(['--candidate-root',str(c),'--expected-root-authority','0'*64,'--expected-runtime-identity',i])==1; assert json.loads(capsys.readouterr().out)['overall']=='FAIL'


@pytest.mark.parametrize(
    ('mutation', 'stage', 'code'),
    [
        ('runtime_identity', 'runtime_projection', 'RUNTIME_IDENTITY_MISMATCH'),
        ('attachment', 'attachments', 'ATTACHMENT_RECORD'),
        ('focused', 'focused_evidence', 'FOCUSED_RECORD'),
        ('writable', 'candidate_inventory', 'INVENTORY_FILE_POLICY'),
    ],
    ids=lambda value: str(value),
)
def test_material_contract_mutations_fail(tmp_path, mutation, stage, code):
    candidate, authority, identity = _fixture(tmp_path)
    if mutation == 'runtime_identity':
        identity = 'f' * 64
    elif mutation == 'attachment':
        _mutate_json(
            candidate, 'manifests/attachments.json',
            lambda value: value['attachments'][0].__setitem__('size', 0),
        )
        authority = _reseal(candidate)
    elif mutation == 'focused':
        _mutate_json(
            candidate, 'manifests/focused_results.json',
            lambda value: value['cases'][0].__setitem__('passed', False),
        )
        authority = _reseal(candidate)
    else:
        (candidate / 'materialization/runtime_root/config/robot.yaml').chmod(0o644)
    before = len(os.listdir('/proc/self/fd'))
    result = validate_frozen_candidate(
        candidate, expected_root_authority=authority,
        expected_runtime_identity=identity,
    )
    _assert_single_failure(result, stage, code)
    assert len(os.listdir('/proc/self/fd')) == before

@pytest.mark.parametrize(
    ('kind', 'stage', 'code'),
    [
        ('symlink_member', 'candidate_inventory', 'CANDIDATE_OPEN'),
        ('hardlink_member', 'candidate_inventory', 'INVENTORY_FILE_POLICY'),
        ('extra_member', 'candidate_inventory', 'INVENTORY_SET_MISMATCH'),
        ('unsafe_bundle', 'candidate_inventory', 'BUNDLE_FIXED_PATH'),
        ('unknown_bundle', 'candidate_inventory', 'BUNDLE_SCHEMA'),
        ('duplicate_key', 'candidate_inventory', 'JSON_DUPLICATE_KEY'),
        ('bad_closure', 'report_and_static_closure', 'STATIC_CLOSURE_SCHEMA'),
        ('bad_category', 'focused_evidence', 'FOCUSED_SCHEMA'),
        ('bad_provenance', 'focused_evidence', 'FOCUSED_RECORD'),
        ('bad_attachment', 'attachments', 'ATTACHMENT_RECORD'),
    ],
    ids=lambda value: str(value),
)
def test_security_regressions_rejected(tmp_path, kind, stage, code):
    candidate, authority, identity = _fixture(tmp_path)
    if kind == "symlink_member":
        parent = candidate / 'materialization/runtime_root/config'; parent.chmod(0o755)
        target = parent / 'robot.yaml'; target.unlink(); os.symlink('robot.yaml', target)
        parent.chmod(0o555)
    elif kind == "hardlink_member":
        parent = candidate / 'focused_raw'; parent.chmod(0o755)
        os.link(candidate/'focused_raw/case-001/attachment.bin', parent/'alias.bin')
        parent.chmod(0o555)
    elif kind == "extra_member":
        candidate.chmod(0o755)
        path = candidate/'extra.bin'; path.write_bytes(b'extra'); path.chmod(0o444)
        candidate.chmod(0o555)
    elif kind == "unsafe_bundle":
        _mutate_json(candidate, 'manifests/validate_only_bundle.json',
                     lambda value: value.__setitem__('inventory_path', '../escape.json'))
        authority = _reseal(candidate)
    elif kind == "unknown_bundle":
        _mutate_json(candidate, 'manifests/validate_only_bundle.json',
                     lambda value: value.__setitem__('unknown', 1))
        authority = _reseal(candidate)
    elif kind == "duplicate_key":
        path = candidate/'manifests/validate_only_bundle.json'; path.chmod(0o644)
        raw = _canonical(json.loads(path.read_text())).decode('utf-8')
        path.write_text(raw[:-1] + ',"schema":"ctr-frozen-candidate-bundle-1"}')
        path.chmod(0o444); authority = _reseal(candidate)
    elif kind == "bad_closure":
        path = candidate/'manifests/static_closure.json'; path.chmod(0o644)
        path.write_bytes(_canonical({'failed_checks': 0, 'check_count': 991})); path.chmod(0o444)
        authority = _reseal(candidate)
    elif kind == "bad_category":
        _mutate_json(candidate, 'manifests/focused_results.json',
                     lambda value: value['category_totals'].__setitem__('authority', 1))
        authority = _reseal(candidate)
    elif kind == "bad_provenance":
        _mutate_json(candidate, 'manifests/focused_results.json',
                     lambda value: value['cases'][0].__setitem__('validator_origin', 'invalid'))
        authority = _reseal(candidate)
    else:
        _mutate_json(candidate, 'manifests/attachments.json',
                     lambda value: value['attachments'][0].__setitem__('size', 0))
        authority = _reseal(candidate)
    before = len(os.listdir('/proc/self/fd'))
    result = validate_frozen_candidate(
        candidate, expected_root_authority=authority,
        expected_runtime_identity=identity,
    )
    _assert_single_failure(result, stage, code)
    assert len(os.listdir('/proc/self/fd')) == before

def test_malformed_direct_root_is_stable(tmp_path):
    r=validate_frozen_candidate(object(), expected_root_authority='0'*64, expected_runtime_identity='1'*64)
    assert r.overall=='FAIL'

def test_repeated_output_is_deterministic(tmp_path, capsys):
    c,a,i=_fixture(tmp_path); assert main(['--candidate-root',str(c),'--expected-root-authority',a,'--expected-runtime-identity',i])==0; first=capsys.readouterr().out
    assert main(['--candidate-root',str(c),'--expected-root-authority',a,'--expected-runtime-identity',i])==0; assert first==capsys.readouterr().out

@pytest.mark.parametrize("mutation", ["missing_category","missing_origin","missing_symbol","missing_raw","unknown_field","wrong_total","pairing","duplicate_id","missing_manifest","size","digest","case_id","missing_role","duplicate_role","member_digest","identity","extra","shared","legacy","structured"], ids=lambda x:x)
def test_phase_c1_strict_contract_rejections(tmp_path, mutation):
    c,_,i=_fixture(tmp_path); p=c/'manifests/focused_results.json'; c.chmod(0o755); p.chmod(0o644); x=json.loads(p.read_text()); case=x['cases'][0]
    expected='FOCUSED_RECORD'
    if mutation=='missing_category': case.pop('category')
    elif mutation=='missing_origin': case.pop('validator_origin')
    elif mutation=='missing_symbol': case.pop('validator_symbol')
    elif mutation=='missing_raw': case.pop('raw_package')
    elif mutation=='unknown_field': case['unexpected']=1
    elif mutation=='wrong_total': x['category_totals']['authority']=42; expected='FOCUSED_SCHEMA'
    elif mutation=='pairing': case['validator_origin']='candidate_evidence_integrity'
    elif mutation=='duplicate_id': x['cases'][1]['case_id']=case['case_id']; expected='RAW_PACKAGE_DIRECTORY'
    elif mutation=='missing_manifest': case['raw_package']['manifest_path']='focused_raw/case-999/manifest.json'; expected='FILE_MISSING'
    elif mutation=='size': case['raw_package']['manifest_size']+=1; expected='RAW_PACKAGE_BINDING'
    elif mutation=='digest': case['raw_package']['manifest_sha256']='0'*64; expected='RAW_PACKAGE_BINDING'
    elif mutation in {'case_id','missing_role','duplicate_role','member_digest','identity','extra'}:
        p.write_bytes(_canonical(x)); p.chmod(0o444)
        raw_path=c/'focused_raw/case-001/manifest.json'; raw=json.loads(raw_path.read_text())
        if mutation=='case_id':
            _rewrite_raw_package(c,'case-001',lambda value:value.__setitem__('case_id','case-999')); expected='RAW_PACKAGE_SCHEMA'
        elif mutation=='missing_role':
            (c/'focused_raw/case-001').chmod(0o755)
            removed=raw['members'][0]; (c/removed['path']).unlink()
            _rewrite_raw_package(c,'case-001',lambda value:value['members'].pop(0)); expected='RAW_PACKAGE_ROLES'
        elif mutation=='duplicate_role':
            role=raw['members'][0]['role']
            _rewrite_raw_package(c,'case-001',lambda value:value['members'][1].__setitem__('role',role)); expected='RAW_PACKAGE_MEMBER'
        elif mutation=='member_digest':
            _rewrite_raw_package(c,'case-001',lambda value:value['members'][0].__setitem__('sha256','0'*64)); expected='RAW_PACKAGE_MEMBER'
        elif mutation=='identity':
            _rewrite_raw_package(c,'case-001',lambda value:value.__setitem__('package_identity','0'*64),recompute_identity=False); expected='RAW_PACKAGE_IDENTITY'
        else:
            (c/'focused_raw/case-001').chmod(0o755)
            extra=c/'focused_raw/case-001/undeclared.json'; extra.write_bytes(b'{}'); extra.chmod(0o444); expected='RAW_PACKAGE_ROLES'
        authority=_reseal(c)
        result=validate_frozen_candidate(c,expected_root_authority=authority,expected_runtime_identity=i)
        _assert_single_failure(result,'focused_evidence',expected)
        return
    elif mutation=='shared': x['cases'][1]['raw_package']=case['raw_package'].copy(); expected='RAW_PACKAGE_DIRECTORY'
    elif mutation=='legacy': x['cases'][0]={'case_id':case['case_id'],'passed':True}
    else: case['passed']='true'
    p.write_bytes(_canonical(x)); p.chmod(0o444)
    authority=_reseal(c)
    result=validate_frozen_candidate(c,expected_root_authority=authority,expected_runtime_identity=i)
    _assert_single_failure(result,'focused_evidence',expected)

@pytest.mark.parametrize("mutation", ["missing","extra","dup_id","dup_path","auth_case","alloc_case","receipt_case","child_case","role","raw_role","forward","reverse","raw_missing","non_designated","multiple","outside","absent_inventory","size","digest","zero"], ids=lambda x:x)
def test_phase_c2_attachment_bijection_rejections(tmp_path, mutation):
    c,_,i=_fixture(tmp_path); p=c/'manifests/attachments.json'; c.chmod(0o755); p.chmod(0o644); x=json.loads(p.read_text()); aa=x['attachments']
    expected='ATTACHMENT_BINDING'
    if mutation=='missing': aa.pop(); expected='ATTACHMENT_SCHEMA'
    elif mutation=='extra': aa.append(dict(aa[-1])); expected='ATTACHMENT_SCHEMA'
    elif mutation=='dup_id': aa[1]['attachment_id']=aa[0]['attachment_id']
    elif mutation=='dup_path':
        aa[1]['path']=aa[0]['path']; aa[1]['size']=aa[0]['size']; aa[1]['sha256']=aa[0]['sha256']; expected='ATTACHMENT_MISMATCH'
    elif mutation in {'auth_case','alloc_case','receipt_case','child_case'}: aa[0 if mutation=='auth_case' else 8 if mutation=='alloc_case' else 16 if mutation=='receipt_case' else 24]['case_id']='case-999'
    elif mutation=='role': aa[0]['role']='allocation'
    elif mutation=='raw_role':
        _rewrite_raw_package(c,'case-001',lambda raw:next(member for member in raw['members'] if member['role']=='attachment_authorization').__setitem__('role','attachment_allocation'))
        expected='ATTACHMENT_RAW_PACKAGE'
    elif mutation=='forward':
        fx=c/'manifests/focused_results.json'; fx.chmod(0o644); f=json.loads(fx.read_text()); f['cases'][0]['attachment_ids']=[]; fx.write_bytes(_canonical(f)); fx.chmod(0o444); expected='ATTACHMENT_FORWARD_REFERENCE'
    elif mutation=='reverse': aa[0]['case_id']='case-002'
    elif mutation=='raw_missing':
        package_dir=c/'focused_raw/case-001'; package_dir.chmod(0o755)
        attachment=package_dir/'attachment.bin'; attachment.unlink()
        _rewrite_raw_package(c,'case-001',lambda raw:raw.__setitem__('members',[member for member in raw['members'] if member['role']!='attachment_authorization']))
        expected='FILE_MISSING'
    elif mutation=='non_designated':
        package_dir=c/'focused_raw/case-009'; package_dir.chmod(0o755)
        extra=package_dir/'unexpected_attachment.bin'; extra.write_bytes(b'extra'); extra.chmod(0o444)
        record={'path':extra.relative_to(c).as_posix(),'role':'attachment_authorization','size':5,'sha256':sha256(b'extra').hexdigest()}
        _rewrite_raw_package(c,'case-009',lambda raw:raw['members'].append(record)); expected='ATTACHMENT_RAW_PACKAGE'
    elif mutation=='multiple':
        package_dir=c/'focused_raw/case-001'; package_dir.chmod(0o755)
        extra=package_dir/'second_attachment.bin'; extra.write_bytes(b'second'); extra.chmod(0o444)
        record={'path':extra.relative_to(c).as_posix(),'role':'attachment_allocation','size':6,'sha256':sha256(b'second').hexdigest()}
        _rewrite_raw_package(c,'case-001',lambda raw:raw['members'].append(record))
        fx=c/'manifests/focused_results.json'; fx.chmod(0o644); f=json.loads(fx.read_text()); f['cases'][0]['attachment_ids'].append('EXTRA-01'); fx.write_bytes(_canonical(f)); fx.chmod(0o444)
        expected='ATTACHMENT_FORWARD_REFERENCE'
    elif mutation=='outside':
        source=aa[1]; aa[0]['path']=source['path']; aa[0]['size']=source['size']; aa[0]['sha256']=source['sha256']; expected='ATTACHMENT_RAW_PACKAGE'
    elif mutation=='absent_inventory': aa[0]['path']='focused_raw/case-001/missing.bin'; expected='FILE_MISSING'
    elif mutation=='size': aa[0]['size']+=1; expected='ATTACHMENT_MISMATCH'
    elif mutation=='digest': aa[0]['sha256']='0'*64; expected='ATTACHMENT_MISMATCH'
    elif mutation=='zero':
        attachment=c/aa[0]['path']; attachment.chmod(0o644); attachment.write_bytes(b''); attachment.chmod(0o444)
        def zero_member(raw):
            member=next(item for item in raw['members'] if item['role']=='attachment_authorization')
            member['size']=0; member['sha256']=sha256(b'').hexdigest()
        _rewrite_raw_package(c,'case-001',zero_member)
        aa[0]['size']=0; aa[0]['sha256']=sha256(b'').hexdigest(); expected='ATTACHMENT_RECORD'
    p.write_bytes(_canonical(x)); p.chmod(0o444)
    authority=_reseal(c)
    result=validate_frozen_candidate(c,expected_root_authority=authority,expected_runtime_identity=i)
    _assert_single_failure(result,'attachments',expected)

def test_attachment_equal_bytes_at_distinct_paths_and_inodes_are_accepted(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    first = candidate / 'focused_raw/case-001/attachment.bin'
    second = candidate / 'focused_raw/case-002/attachment.bin'
    second.chmod(0o644); second.write_bytes(first.read_bytes()); second.chmod(0o444)
    digest = sha256(second.read_bytes()).hexdigest()
    def update_member(raw):
        member = next(item for item in raw['members'] if item['role'] == 'attachment_authorization')
        member['size'] = second.stat().st_size; member['sha256'] = digest
    _rewrite_raw_package(candidate, 'case-002', update_member)
    _mutate_json(candidate, 'manifests/attachments.json', lambda value: value['attachments'][1].update({'size': second.stat().st_size, 'sha256': digest}))
    authority = _reseal(candidate)
    authority = _refresh_closure_observations(candidate, authority, identity)
    assert first.read_bytes() == second.read_bytes()
    assert first.stat().st_ino != second.stat().st_ino
    result = validate_frozen_candidate(candidate, expected_root_authority=authority, expected_runtime_identity=identity)
    assert result.overall == 'PASS'


def test_attachment_hardlink_is_rejected_by_physical_inventory(tmp_path):
    candidate, authority, identity = _fixture(tmp_path)
    package = candidate / 'focused_raw/case-001'; package.chmod(0o755)
    os.link(package / 'attachment.bin', package / 'alias.bin'); package.chmod(0o555)
    result = validate_frozen_candidate(candidate, expected_root_authority=authority, expected_runtime_identity=identity)
    _assert_single_failure(result, 'candidate_inventory', 'INVENTORY_FILE_POLICY')


def test_attachment_failure_preserves_descriptor_count(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    _mutate_json(candidate, 'manifests/attachments.json', lambda value: value['attachments'][0].__setitem__('size', 0))
    authority = _reseal(candidate); before = len(os.listdir('/proc/self/fd'))
    result = validate_frozen_candidate(candidate, expected_root_authority=authority, expected_runtime_identity=identity)
    _assert_single_failure(result, 'attachments', 'ATTACHMENT_RECORD')
    assert len(os.listdir('/proc/self/fd')) == before


def test_attachment_cli_failure_is_structured(tmp_path, capsys):
    candidate, _, identity = _fixture(tmp_path)
    _mutate_json(candidate, 'manifests/attachments.json', lambda value: value['attachments'].pop())
    authority = _reseal(candidate)
    assert main(['--candidate-root', str(candidate), '--expected-root-authority', authority,
                 '--expected-runtime-identity', identity]) == 1
    output = json.loads(capsys.readouterr().out)
    assert output['overall'] == 'FAIL'
    assert [trace for trace in output['traces'] if trace['status'] == 'FAIL'] == [
        next(trace for trace in output['traces'] if trace['name'] == 'attachments')
    ]


@pytest.mark.parametrize(
    ('field', 'type_name'),
    [('bundle', '_BundleFacts'), ('inventory', '_InventoryFacts'), ('authority', '_AuthorityFacts'),
     ('runtime', '_RuntimeFacts'), ('plans', 'tuple'), ('focused', '_FocusedFacts'),
     ('attachments', '_AttachmentFacts'), ('capsule', '_CapsuleFact'),
     ('report_inputs', '_ReportInputFacts'), ('invocation', '_InvocationFacts')],
    ids=lambda value: str(value),
)
def test_refactor_stage_outputs_are_typed_facts(tmp_path, field, type_name):
    candidate, authority, identity = _fixture(tmp_path)
    facts = _upstream_facts(candidate, authority, identity)
    value = getattr(facts, field)
    assert type(value).__name__ == type_name


def test_candidate_facts_and_nested_values_are_immutable(tmp_path):
    candidate, authority, identity = _fixture(tmp_path)
    facts = _upstream_facts(candidate, authority, identity)
    with pytest.raises(AttributeError): facts.runtime = None
    with pytest.raises(TypeError): facts.runtime.graph[0] = ('changed', True)
    with pytest.raises(AttributeError): facts.plans[0].embedded_runtime_identity = '0' * 64


@pytest.mark.parametrize(
    ('field', 'value'),
    [('path', '../escape'), ('size', True), ('sha256', 'not-a-digest'), ('mode', '0999')],
    ids=['unsafe_path', 'boolean_size', 'bad_digest', 'bad_mode'],
)
def test_file_fact_constructor_rejects_invalid_values(field, value):
    arguments = {'path': 'reports/report.md', 'size': 1, 'sha256': '0' * 64, 'mode': '0444'}
    arguments[field] = value
    with pytest.raises(candidate_module.CandidateValidateOnlyError):
        candidate_module._FileFact(**arguments)


def test_inventory_fact_detaches_caller_list_and_rejects_duplicates(tmp_path):
    candidate, authority, identity = _fixture(tmp_path)
    facts = _upstream_facts(candidate, authority, identity)
    caller = list(facts.inventory.members)
    detached = candidate_module._InventoryFacts(facts.inventory.file, caller)
    caller.pop()
    assert len(detached.members) == len(facts.inventory.members)
    with pytest.raises(candidate_module.CandidateValidateOnlyError):
        candidate_module._InventoryFacts(facts.inventory.file, [facts.inventory.members[0]] * 2)


def test_authority_fact_requires_complete_v2_child_role_set(tmp_path):
    candidate, authority, identity = _fixture(tmp_path)
    facts = _upstream_facts(candidate, authority, identity)
    with pytest.raises(candidate_module.CandidateValidateOnlyError):
        replace(facts.authority, children=facts.authority.children[:-1])


@pytest.mark.parametrize('field', ['members', 'dependencies'])
def test_runtime_fact_rejects_invalid_structural_counts(tmp_path, field):
    candidate, authority, identity = _fixture(tmp_path)
    runtime = _upstream_facts(candidate, authority, identity).runtime
    with pytest.raises(candidate_module.CandidateValidateOnlyError):
        replace(runtime, **{field: getattr(runtime, field)[:-1]})


def test_plan_fact_requires_three_way_runtime_identity_equality(tmp_path):
    candidate, authority, identity = _fixture(tmp_path)
    plan = _upstream_facts(candidate, authority, identity).plans[0]
    with pytest.raises(candidate_module.CandidateValidateOnlyError):
        replace(plan, embedded_runtime_identity='f' * 64)


def test_focused_and_attachment_fact_counts_are_constructor_invariants(tmp_path):
    candidate, authority, identity = _fixture(tmp_path)
    facts = _upstream_facts(candidate, authority, identity)
    with pytest.raises(candidate_module.CandidateValidateOnlyError):
        replace(facts.focused, cases=facts.focused.cases[:-1])
    with pytest.raises(candidate_module.CandidateValidateOnlyError):
        replace(facts.attachments, records=facts.attachments.records[:-1])


@pytest.mark.parametrize(
    ('key', 'category', 'paths'),
    [('', 'candidate_inventory', ()),
     ('candidate.inventory', 'invalid', ()),
     ('candidate.inventory', 'candidate_inventory', ('/absolute',))],
    ids=['empty_key', 'invalid_category', 'absolute_source'],
)
def test_observation_constructor_rejects_invalid_authority(key, category, paths):
    with pytest.raises(candidate_module.CandidateValidateOnlyError):
        candidate_module._Observation(key, category, {'value': 1}, paths)


def test_observation_deeply_detaches_nested_caller_values():
    payload = {'nested': [1]}
    observation = candidate_module._Observation(
        'candidate.inventory', 'candidate_inventory', payload,
        ('manifests/candidate_inventory.json',),
    )
    payload['nested'].append(2)
    assert candidate_module._thaw(observation.value) == {'nested': [1]}


def test_candidate_aggregate_rejects_wrong_subrecord_type(tmp_path):
    candidate, authority, identity = _fixture(tmp_path)
    facts = _upstream_facts(candidate, authority, identity)
    with pytest.raises(candidate_module.CandidateValidateOnlyError):
        replace(facts, runtime=facts.inventory)


def test_runtime_results_are_retained_and_observations_derive_from_them(tmp_path):
    candidate, authority, identity = _fixture(tmp_path)
    facts = _upstream_facts(candidate, authority, identity)
    assert type(facts.runtime.projection_reconciliation).__name__ == 'RuntimeProjectionReconciliation'
    assert type(facts.runtime.dependency_closure).__name__ == 'RuntimeDependencyClosure'
    observations = candidate_module._build_required_observations(facts)
    physical = candidate_module._thaw(observations['runtime.physical'].value)
    dependencies = candidate_module._thaw(observations['runtime.dependencies'].value)
    assert physical['matched_count'] == facts.runtime.projection_reconciliation.matched_count
    assert physical['issue_count'] == len(facts.runtime.projection_reconciliation.issues)
    assert dependencies['reachable_member_count'] == len(facts.runtime.dependency_closure.reachable_members)
    assert dependencies['unresolved'] == len(facts.runtime.dependency_closure.issues)


def test_nonclean_retained_runtime_result_is_rejected(tmp_path):
    candidate, authority, identity = _fixture(tmp_path)
    runtime = _upstream_facts(candidate, authority, identity).runtime
    bad = replace(runtime.projection_reconciliation, matched_count=171,
                  issues=(RuntimeIssue('PHYSICAL_CHANGED_DURING_READ'),))
    with pytest.raises(candidate_module.CandidateValidateOnlyError):
        replace(runtime, projection_reconciliation=bad)


@pytest.mark.parametrize('fact_kind', ['raw_package', 'focused_aggregate', 'attachment_aggregate'])
def test_fact_aggregate_constructor_invariants(tmp_path, fact_kind):
    candidate, authority, identity = _fixture(tmp_path)
    facts = _upstream_facts(candidate, authority, identity)
    with pytest.raises(candidate_module.CandidateValidateOnlyError):
        if fact_kind == 'raw_package':
            replace(facts.focused.cases[0].package, package_identity='0' * 64)
        elif fact_kind == 'focused_aggregate':
            replace(facts.focused, package_aggregate_sha256='0' * 64)
        else:
            aggregates = list(facts.attachments.role_aggregates)
            aggregates[0] = (aggregates[0][0], '0' * 64)
            replace(facts.attachments, role_aggregates=aggregates)


def test_six_validated_runtime_plan_objects_are_retained(tmp_path):
    candidate, authority, identity = _fixture(tmp_path)
    facts = _upstream_facts(candidate, authority, identity)
    assert len(facts.plans) == 6
    assert all(type(item.plan).__name__ == 'RuntimePlan' for item in facts.plans)
    assert {item.plan.production_runtime_identity for item in facts.plans} == {identity}


def test_required_observations_are_exact_and_read_only(tmp_path):
    candidate, authority, identity = _fixture(tmp_path)
    observations = candidate_module._build_required_observations(_upstream_facts(candidate, authority, identity))
    assert len(observations) == 35
    assert len({item.category for item in observations.values()}) == 35
    assert not any(item.value in (None, (), {}) for item in observations.values())
    with pytest.raises(TypeError): observations['new'] = object()


def test_report_mutation_cannot_replace_upstream_observations(tmp_path):
    candidate, authority, identity = _fixture(tmp_path)
    before = candidate_module._build_required_observations(_upstream_facts(candidate, authority, identity))
    _mutate_json(candidate, 'manifests/report_source.json', lambda value: value['runtime'].__setitem__('member_count', 1))
    authority = _reseal(candidate)
    after = candidate_module._build_required_observations(_upstream_facts(candidate, authority, identity))
    assert before == after


def test_closure_mutation_cannot_replace_candidate_facts(tmp_path):
    candidate, authority, identity = _fixture(tmp_path)
    before = candidate_module._build_required_observations(_upstream_facts(candidate, authority, identity))
    _mutate_json(candidate, 'manifests/static_closure.json', lambda value: value.__setitem__('failed_checks', 1))
    authority = _reseal(candidate)
    after = candidate_module._build_required_observations(_upstream_facts(candidate, authority, identity))
    assert before == after


def test_report_input_files_are_distinct_cached_facts(tmp_path):
    candidate, authority, identity = _fixture(tmp_path)
    report_inputs = _upstream_facts(candidate, authority, identity).report_inputs
    paths = {report_inputs.report_source.path, report_inputs.static_closure.path,
             report_inputs.correction_report.path, report_inputs.source_identity.path,
             report_inputs.predecessor_preservation.path}
    assert len(paths) == 5
    assert all(getattr(report_inputs, name).sha256
               for name in ('report_source', 'static_closure', 'correction_report', 'source_identity', 'predecessor_preservation'))


def test_positive_closure_has_exact_two_tier_structure(tmp_path):
    candidate, _, _ = _fixture(tmp_path)
    closure = json.loads((candidate / 'manifests/static_closure.json').read_text())
    assert len(closure['checks']) == closure['check_count'] == 1154
    assert sum(check['kind'] == 'critical_observation' for check in closure['checks']) == 35
    assert sum(check['kind'] == 'authenticated_source' for check in closure['checks']) == 1119
    categories = {
        check['category'] if check['kind'] == 'critical_observation'
        else check['semantic_category']
        for check in closure['checks']
    }
    assert len(categories) == closure['category_count'] == 46


def test_current_958_copy_pattern_is_rejected(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    closure_path = 'manifests/static_closure.json'
    def pad(value):
        critical = [record for record in value['checks'] if record['kind'] == 'critical_observation']
        template = next(record for record in value['checks'] if record['kind'] == 'authenticated_source')
        copies = []
        for index in range(958):
            record = json.loads(json.dumps(template))
            if index:
                record['check_id'] = f'legacy-source-{index + 1:03d}'
            copies.append(record)
        value['checks'] = critical + copies
        value['authenticated_source_count'] = 958
        value['check_count'] = 992
    _mutate_json(candidate, closure_path, pad)
    authority = _reseal(candidate)
    result = validate_frozen_candidate(candidate, expected_root_authority=authority,
                                       expected_runtime_identity=identity)
    _assert_single_failure(result, 'report_and_static_closure',
                           'CLOSURE_DUPLICATE_SEMANTIC_SOURCE_ASSERTION')


def test_duplicate_source_record_with_different_check_id_is_rejected(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    def duplicate(value):
        record = json.loads(json.dumps(value['checks'][35]))
        record['check_id'] = 'different-id-same-semantic-assertion'
        value['checks'].append(record)
        value['authenticated_source_count'] += 1
        value['check_count'] += 1
    _mutate_json(candidate, 'manifests/static_closure.json', duplicate)
    authority = _reseal(candidate)
    result = validate_frozen_candidate(candidate, expected_root_authority=authority,
                                       expected_runtime_identity=identity)
    _assert_single_failure(result, 'report_and_static_closure',
                           'CLOSURE_DUPLICATE_SEMANTIC_SOURCE_ASSERTION')


def test_duplicate_path_digest_assertion_in_projection_is_rejected(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    def duplicate(value):
        record = json.loads(json.dumps(value['facts'][0]))
        record['source_fact_id'] = 'physical-file:meaningless-alias'
        value['facts'].append(record)
        value['fact_count'] += 1
        value['category_counts'][record['semantic_category']] += 1
    _mutate_json(candidate, 'manifests/closure_source_projection.json', duplicate)
    authority = _reseal_authority_only(candidate)
    result = validate_frozen_candidate(candidate, expected_root_authority=authority,
                                       expected_runtime_identity=identity)
    _assert_single_failure(result, 'report_and_static_closure',
                           'CLOSURE_DUPLICATE_SEMANTIC_SOURCE_ASSERTION')


def test_missing_projected_source_fact_is_rejected(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    def remove(value):
        record = value['facts'].pop()
        value['fact_count'] -= 1
        value['category_counts'][record['semantic_category']] -= 1
    _mutate_json(candidate, 'manifests/closure_source_projection.json', remove)
    authority = _reseal_authority_only(candidate)
    result = validate_frozen_candidate(candidate, expected_root_authority=authority,
                                       expected_runtime_identity=identity)
    _assert_single_failure(result, 'report_and_static_closure', 'CLOSURE_MISSING_SOURCE_FACT')


@pytest.mark.parametrize(
    'mutation,code',
    [
        ('extra', 'CLOSURE_EXTRA_SOURCE_FACT'),
        ('unknown', 'CLOSURE_UNKNOWN_SOURCE_FACT'),
        ('category', 'CLOSURE_SOURCE_PROJECTION_MISMATCH'),
        ('physical', 'CLOSURE_PHYSICAL_SOURCE_MISMATCH'),
        ('tautology', 'CLOSURE_MANIFEST_CLAIM_TAUTOLOGY'),
        ('self', 'CLOSURE_DEPENDENCY_CYCLE'),
    ],
)
def test_strict_closure_source_record_failures(tmp_path, mutation, code):
    candidate, _, identity = _fixture(tmp_path)
    def alter(value):
        source = value['checks'][35]
        if mutation == 'extra':
            record = json.loads(json.dumps(source))
            record['check_id'] = 'source:physical-file:not-projected'
            record['source_fact_id'] = 'physical-file:not-projected'
            record['source_path'] = 'not-projected'
            value['checks'].append(record); value['check_count'] += 1
            value['authenticated_source_count'] += 1
        elif mutation == 'unknown':
            source['source_path'] = 'unknown-source-path'
        elif mutation == 'category':
            source['semantic_category'] = 'meaningless-alias'
        elif mutation == 'physical':
            source['sha256'] = '0' * 64
            source['expected']['sha256'] = '0' * 64
            source['observed']['sha256'] = '0' * 64
        elif mutation == 'tautology':
            source['derivation_rule'] = 'manifest-claim-equals-identical-manifest-claim'
        else:
            source['source_path'] = 'manifests/static_closure.json'
    _mutate_json(candidate, 'manifests/static_closure.json', alter)
    authority = _reseal(candidate)
    result = validate_frozen_candidate(candidate, expected_root_authority=authority,
                                       expected_runtime_identity=identity)
    _assert_single_failure(result, 'report_and_static_closure', code)


def test_inventory_root_authority_closure_cycle_is_rejected(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    closure = _file_record(candidate, 'manifests/static_closure.json')
    closure['mode'] = '0444'
    _mutate_json(candidate, 'manifests/candidate_inventory.json',
                 lambda value: value['files'].append(closure))
    authority = _reseal_authority_only(candidate)
    result = validate_frozen_candidate(candidate, expected_root_authority=authority,
                                       expected_runtime_identity=identity)
    _assert_single_failure(result, 'candidate_inventory', 'CLOSURE_DEPENDENCY_CYCLE')


def test_inventory_reordering_preserves_canonical_source_projection(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    _mutate_json(candidate, 'manifests/candidate_inventory.json',
                 lambda value: value['files'].reverse())
    authority = _reseal_authority_only(candidate)
    result = validate_frozen_candidate(candidate, expected_root_authority=authority,
                                       expected_runtime_identity=identity)
    assert result.overall == 'PASS'


def test_source_universe_count_is_derived_and_changes_with_universe(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    closure = json.loads((candidate / 'manifests/static_closure.json').read_text())
    assert closure['authenticated_source_count'] == 1119
    reports = candidate / 'reports'; reports.chmod(0o755)
    added = reports / 'new-independent-source.txt'; added.write_bytes(b'new source\n'); added.chmod(0o444)
    reports.chmod(0o555)
    authority = _reseal(candidate)
    authority = _refresh_closure_observations(candidate, authority, identity)
    updated = json.loads((candidate / 'manifests/static_closure.json').read_text())
    assert updated['authenticated_source_count'] == 1120
    assert updated['check_count'] == 1155
    result = validate_frozen_candidate(candidate, expected_root_authority=authority,
                                       expected_runtime_identity=identity)
    assert result.overall == 'PASS'


def test_arbitrary_padding_cannot_restore_stale_992_total(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    def stale(value):
        value['authenticated_source_count'] = 958
        value['check_count'] = 992
    _mutate_json(candidate, 'manifests/static_closure.json', stale)
    authority = _reseal(candidate)
    result = validate_frozen_candidate(candidate, expected_root_authority=authority,
                                       expected_runtime_identity=identity)
    _assert_single_failure(result, 'report_and_static_closure',
                           'CLOSURE_REPORT_COUNT_MISMATCH')


def test_legacy_duplicated_candidate_schema_fails_stably(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    _mutate_json(candidate, 'manifests/validate_only_bundle.json',
                 lambda value: value.__setitem__('schema', 'ctr-frozen-candidate-bundle-1'))
    authority = _reseal(candidate)
    result = validate_frozen_candidate(candidate, expected_root_authority=authority,
                                       expected_runtime_identity=identity)
    _assert_single_failure(result, 'candidate_inventory', 'LEGACY_DUPLICATED_CLOSURE')


def test_future_source_identity_and_diagnostic_lineage_pass(tmp_path):
    candidate, authority, identity = _fixture(tmp_path)
    source = json.loads((candidate / 'manifests/source_identity.json').read_text())
    assert source['schema_version'] == 'ctr-source-identity-2'
    assert source['historical_lineage']['operative'] is False
    assert {item['status'] for item in source['historical_lineage']['superseded_identities']} == {'diagnostic_only'}
    result = validate_frozen_candidate(candidate, expected_root_authority=authority,
                                       expected_runtime_identity=identity)
    assert result.overall == 'PASS'


def test_one_unique_projected_fact_maps_to_one_closure_record(tmp_path):
    candidate, _, _ = _fixture(tmp_path)
    projection = json.loads((candidate / 'manifests/closure_source_projection.json').read_text())
    closure = json.loads((candidate / 'manifests/static_closure.json').read_text())
    projected = {record['source_fact_id']: record for record in projection['facts']}
    source_records = [record for record in closure['checks'] if record['kind'] == 'authenticated_source']
    closed = {record['source_fact_id']: record for record in source_records}
    assert len(projected) == len(closed) == projection['fact_count'] == 1119
    assert set(projected) == set(closed)
    assert len({record['source_path'] for record in source_records}) == len(source_records)
    assert all(record['check_id'] == 'source:' + record['source_fact_id'] for record in source_records)


def test_report_source_uses_only_derived_closure_counts(tmp_path):
    candidate, _, _ = _fixture(tmp_path)
    projection = json.loads((candidate / 'manifests/closure_source_projection.json').read_text())
    closure = json.loads((candidate / 'manifests/static_closure.json').read_text())
    report = json.loads((candidate / 'manifests/report_source.json').read_text())
    assert report['closure_source_projection']['fact_count'] == projection['fact_count']
    assert report['static_closure'] == {
        key: closure[key] for key in (
            'critical_count', 'authenticated_source_count', 'check_count',
            'category_count', 'failed_checks')
    }
    assert closure['check_count'] == closure['critical_count'] + projection['fact_count']


@pytest.mark.parametrize(
    'mutation,code',
    [
        ('superseded', 'SUPERSEDED_OPERATIVE_IDENTITY'),
        ('missing_algorithm', 'MISSING_MATERIALIZATION_ALGORITHM'),
        ('logical', 'MATERIALIZATION_LOGICAL_IDENTITY_MISMATCH'),
        ('physical', 'MATERIALIZATION_PHYSICAL_REHASH_MISMATCH'),
        ('projection', 'MATERIALIZATION_PROJECTION_MISMATCH'),
    ],
)
def test_source_identity_materialization_failures_are_stable(tmp_path, mutation, code):
    candidate, _, identity = _fixture(tmp_path)
    def alter(value):
        materialization = value['materialization']
        if mutation == 'superseded':
            materialization['logical_identity'] = next(iter(candidate_module.SUPERSEDED_HISTORICAL_IDENTITIES))
        elif mutation == 'missing_algorithm':
            materialization['logical_algorithm_id'] = 'missing'
        elif mutation == 'logical':
            materialization['logical_identity'] = '0' * 64
        elif mutation == 'physical':
            materialization['physical_rehash'] = '0' * 64
        else:
            materialization['projection_sha256'] = '0' * 64
    _mutate_json(candidate, 'manifests/source_identity.json', alter)
    authority = _reseal(candidate)
    result = validate_frozen_candidate(candidate, expected_root_authority=authority,
                                       expected_runtime_identity=identity)
    _assert_single_failure(result, 'runtime_physical', code)


def test_missing_materialization_projection_fails_at_stable_stage(tmp_path):
    candidate, authority, identity = _fixture(tmp_path)
    manifests = candidate / 'manifests'; manifests.chmod(0o755)
    (manifests / 'materialization_projection.json').unlink(); manifests.chmod(0o555)
    result = validate_frozen_candidate(candidate, expected_root_authority=authority,
                                       expected_runtime_identity=identity)
    _assert_single_failure(result, 'candidate_inventory', 'MATERIALIZATION_PROJECTION_MISSING')


def test_runtime_materialization_membership_mismatch_fails(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    def remove_runtime_member(value):
        value['members'] = [member for member in value['members']
                            if member['path'] != 'runtime_root/config/robot.yaml']
    _mutate_json(candidate, 'manifests/materialization_projection.json', remove_runtime_member)
    authority = _reseal(candidate)
    result = validate_frozen_candidate(candidate, expected_root_authority=authority,
                                       expected_runtime_identity=identity)
    _assert_single_failure(
        result, 'runtime_physical',
        'MATERIALIZATION_PHYSICAL_PROJECTION_MISMATCH',
    )


@pytest.mark.parametrize(
    ('kind', 'expected_code'),
    [
        ('transient', 'MATERIALIZATION_TRANSIENT_PATH'),
        ('syntactically_valid', 'MATERIALIZATION_PHYSICAL_PROJECTION_MISMATCH'),
    ],
)
def test_coherently_resealed_phantom_projection_fails_at_runtime_physical(
    tmp_path, kind, expected_code,
):
    candidate, _, identity = _fixture(tmp_path)
    projection_path = candidate / 'manifests/materialization_projection.json'
    projection = json.loads(projection_path.read_text())
    if kind == 'transient':
        projection['members'].extend([
            {'kind': 'directory', 'mode': '0555', 'path': '__pycache__', 'role': 'directory'},
            {'kind': 'regular_file', 'mode': '0444', 'path': '__pycache__/ghost.pyc',
             'role': 'regular_file', 'sha256': sha256(b'ghost').hexdigest(), 'size': 5},
        ])
    else:
        projection['members'].extend([
            {'kind': 'directory', 'mode': '0555', 'path': 'phantom', 'role': 'directory'},
            {'kind': 'regular_file', 'mode': '0444', 'path': 'phantom/ghost.bin',
             'role': 'regular_file', 'sha256': sha256(b'ghost').hexdigest(), 'size': 5},
        ])
    authority, claimed = _coherently_reseal_projection_claim(
        candidate, projection, identity,
    )
    source = json.loads((candidate / 'manifests/source_identity.json').read_text())
    report = json.loads((candidate / 'manifests/report_source.json').read_text())
    assert source['materialization']['projection_sha256'] == claimed['projection_sha256']
    assert source['materialization']['physical_rehash'] == claimed['physical_rehash']
    assert report['materialization']['projection_sha256'] == claimed['projection_sha256']
    result = validate_frozen_candidate(
        candidate, expected_root_authority=authority,
        expected_runtime_identity=identity,
    )
    _assert_single_failure(result, 'runtime_physical', expected_code)


def test_missing_candidate_materialization_root_fails_at_runtime_physical(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    _mutate_json(
        candidate, 'manifests/validate_only_bundle.json',
        lambda value: value.__setitem__('materialization_root_path', 'missing-materialization'),
    )
    authority = _reseal(candidate)
    result = validate_frozen_candidate(
        candidate, expected_root_authority=authority,
        expected_runtime_identity=identity,
    )
    _assert_single_failure(
        result, 'runtime_physical', 'MATERIALIZATION_PHYSICAL_ROOT_MISSING',
    )


def test_materialization_root_escape_is_rejected_before_resolution(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    _mutate_json(
        candidate, 'manifests/validate_only_bundle.json',
        lambda value: value.__setitem__('materialization_root_path', '../escape'),
    )
    authority = _reseal(candidate)
    result = validate_frozen_candidate(
        candidate, expected_root_authority=authority,
        expected_runtime_identity=identity,
    )
    _assert_single_failure(result, 'candidate_inventory', 'PATH_INVALID')


def test_materialization_root_symlink_is_rejected_by_candidate_inventory(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    candidate.chmod(0o755)
    (candidate / 'material-link').symlink_to('materialization', target_is_directory=True)
    candidate.chmod(0o555)
    result = validate_frozen_candidate(
        candidate, expected_root_authority='0' * 64,
        expected_runtime_identity=identity,
    )
    _assert_single_failure(result, 'candidate_inventory', 'CANDIDATE_OPEN')


@pytest.mark.parametrize(
    'mutation', ['missing', 'extra', 'digest', 'mode'],
)
def test_candidate_materialization_physical_mismatch_is_not_projection_only(
    tmp_path, mutation,
):
    candidate, _, identity = _fixture(tmp_path)
    parent = candidate / 'materialization/material_only'
    target = parent / 'authority.txt'
    parent.chmod(0o755)
    if mutation == 'missing':
        target.unlink()
    elif mutation == 'extra':
        extra = parent / 'extra.txt'; extra.write_bytes(b'extra\n'); extra.chmod(0o444)
    elif mutation == 'digest':
        target.chmod(0o644); target.write_bytes(b'altered material authority\n'); target.chmod(0o444)
    else:
        target.chmod(0o555)
    parent.chmod(0o555)
    authority = _reseal(candidate, normalize_modes=mutation != 'mode')
    result = validate_frozen_candidate(
        candidate, expected_root_authority=authority,
        expected_runtime_identity=identity,
    )
    _assert_single_failure(
        result, 'runtime_physical',
        'MATERIALIZATION_PHYSICAL_PROJECTION_MISMATCH',
    )


@pytest.mark.parametrize(
    ('mutation', 'code'),
    [
        ('same_size', 'MATERIALIZATION_CONCURRENT_MUTATION'),
        ('inode', 'MATERIALIZATION_INODE_SUBSTITUTION'),
    ],
)
def test_candidate_materialization_races_fail_in_physical_trace(
    tmp_path, monkeypatch, mutation, code,
):
    candidate, authority, identity = _fixture(tmp_path)
    original = materialization_module._scan_materialization_at
    calls = 0

    def mutate_between_passes(parent_fd, relative_root):
        nonlocal calls
        result = original(parent_fd, relative_root)
        calls += 1
        if calls == 1:
            parent = candidate / 'materialization/material_only'
            target = parent / 'authority.txt'
            raw = target.read_bytes()
            parent.chmod(0o755)
            if mutation == 'same_size':
                target.chmod(0o644)
                target.write_bytes(bytes([raw[0] ^ 1]) + raw[1:])
                target.chmod(0o444)
            else:
                replacement = parent / 'replacement'
                replacement.write_bytes(raw); replacement.chmod(0o444)
                os.replace(replacement, target)
            parent.chmod(0o555)
        return result

    monkeypatch.setattr(
        materialization_module, '_scan_materialization_at', mutate_between_passes,
    )
    result = validate_frozen_candidate(
        candidate, expected_root_authority=authority,
        expected_runtime_identity=identity,
    )
    _assert_single_failure(result, 'runtime_physical', code)


def test_runtime_root_must_be_inside_verified_materialization(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    alternate = candidate / 'alternate_materialization'
    candidate.chmod(0o755)
    alternate.mkdir(); (alternate / 'empty').mkdir()
    (alternate / 'empty').chmod(0o555); alternate.chmod(0o555); candidate.chmod(0o555)
    projection = build_materialization_projection(alternate).to_dict()
    _mutate_json(
        candidate, 'manifests/validate_only_bundle.json',
        lambda value: value.__setitem__('materialization_root_path', 'alternate_materialization'),
    )
    _mutate_json(
        candidate, 'manifests/source_identity.json',
        lambda value: value['materialization'].__setitem__(
            'materialization_root_path', 'alternate_materialization',
        ),
    )
    _mutate_json(
        candidate, 'manifests/report_source.json',
        lambda value: value['materialization'].__setitem__(
            'materialization_root_path', 'alternate_materialization',
        ),
    )
    authority, _ = _coherently_reseal_projection_claim(candidate, projection, identity)
    result = validate_frozen_candidate(
        candidate, expected_root_authority=authority,
        expected_runtime_identity=identity,
    )
    _assert_single_failure(
        result, 'runtime_physical',
        'MATERIALIZATION_RUNTIME_MEMBERSHIP_MISMATCH',
    )


def test_materialization_subtree_enters_derived_source_universe_once(tmp_path):
    candidate, _, _ = _fixture(tmp_path)
    bundle = json.loads((candidate / 'manifests/validate_only_bundle.json').read_text())
    inventory = json.loads((candidate / bundle['inventory_path']).read_text())
    projection = json.loads((candidate / bundle['closure_source_projection_path']).read_text())
    facts = projection['facts']
    assert projection['fact_count'] == len(inventory['files']) == len(facts)
    assert len({fact['source_path'] for fact in facts}) == len(facts)
    runtime = [fact for fact in facts if fact['semantic_category'] == 'runtime']
    materialization = [fact for fact in facts if fact['semantic_category'] == 'materialization']
    assert len(runtime) == 172
    assert {fact['role'] for fact in runtime} == {'runtime_member'}
    assert {fact['source_path'] for fact in materialization} == {
        'manifests/materialization_projection.json',
        'materialization/material_only/authority.txt',
    }


def test_orchestration_has_no_mutable_cross_stage_state(tmp_path):
    source = (ROOT / 'ctr_bringup/runtime_candidate_validate_only.py').read_text()
    orchestration = source[source.index('def validate_frozen_candidate'):source.index('def _result_dict')]
    assert 'state =' not in orchestration
    assert 'state[' not in orchestration
    assert 'def stage_' not in orchestration


def test_success_trace_order_and_json_are_deterministic(tmp_path, capsys):
    candidate, authority, identity = _fixture(tmp_path)
    argv = ['--candidate-root', str(candidate), '--expected-root-authority', authority,
            '--expected-runtime-identity', identity]
    assert main(argv) == 0; first = capsys.readouterr().out
    assert main(argv) == 0; second = capsys.readouterr().out
    assert first == second
    assert [trace['name'] for trace in json.loads(first)['traces']] == list(candidate_module.TRACE_NAMES)


def test_failed_cli_json_is_deterministic(tmp_path, capsys):
    candidate, _, identity = _fixture(tmp_path)
    argv = ['--candidate-root', str(candidate), '--expected-root-authority', '0' * 64,
            '--expected-runtime-identity', identity]
    assert main(argv) == 1; first = capsys.readouterr().out
    assert main(argv) == 1; second = capsys.readouterr().out
    assert first == second and 'Traceback' not in first


def test_malformed_direct_inputs_are_structured_and_deterministic():
    first = validate_frozen_candidate(object(), expected_root_authority={}, expected_runtime_identity=[])
    second = validate_frozen_candidate(object(), expected_root_authority={}, expected_runtime_identity=[])
    assert first == second
    assert first.overall == 'FAIL' and len(first.traces) == 1
    assert first.traces[0].name == 'candidate_inventory'
    assert sum(trace.status == 'FAIL' for trace in first.traces) == 1


def test_plan_embedded_identity_mutation_fails_at_plan_trace(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    _mutate_json(candidate, 'plans/production_root.json',
                 lambda value: value.__setitem__('production_runtime_identity', 'f' * 64))
    authority = _reseal(candidate)
    result = validate_frozen_candidate(candidate, expected_root_authority=authority, expected_runtime_identity=identity)
    assert result.overall == 'FAIL'
    assert result.traces[5].name == 'six_plan_set' and result.traces[5].status == 'FAIL'


def test_late_closure_failure_preserves_descriptor_count(tmp_path):
    candidate, _, identity = _fixture(tmp_path)
    _mutate_json(candidate, 'manifests/static_closure.json', lambda value: value.__setitem__('failed_checks', 1))
    authority = _reseal(candidate); before = len(os.listdir('/proc/self/fd'))
    for _ in range(4):
        result = validate_frozen_candidate(candidate, expected_root_authority=authority, expected_runtime_identity=identity)
        assert result.overall == 'FAIL'
    assert len(os.listdir('/proc/self/fd')) == before


@pytest.mark.parametrize(
    'mutation',
    ['report_missing', 'report_unknown', 'report_runtime', 'report_plan', 'report_focused',
     'report_attachment', 'report_static', 'report_correction', 'report_capsule', 'report_basename',
     'report_self_digest', 'closure_legacy', 'closure_count', 'critical_missing',
     'critical_duplicate', 'source_digest', 'source_self', 'kind', 'category', 'passed_int',
     'expected', 'source_paths', 'duplicate_check_id', 'closure_self_digest'],
    ids=lambda value: value,
)
def test_coordinated_report_and_closure_contract_rejections(tmp_path, mutation):
    candidate, _, identity = _fixture(tmp_path)
    report_path = 'manifests/report_source.json'; closure_path = 'manifests/static_closure.json'
    if mutation.startswith('report_'):
        def alter_report(value):
            if mutation == 'report_missing': value.pop('runtime')
            elif mutation == 'report_unknown': value['unknown'] = 1
            elif mutation == 'report_runtime': value['runtime']['member_count'] = 171
            elif mutation == 'report_plan': value['plans']['production_root']['sha256'] = '0' * 64
            elif mutation == 'report_focused': value['focused']['case_count'] = 149
            elif mutation == 'report_attachment': value['attachments']['roles']['authorization'] = 7
            elif mutation == 'report_static': value['static_closure']['check_count'] = 991
            elif mutation == 'report_correction': value['correction_report']['size'] += 1
            elif mutation == 'report_capsule': value['capsule']['policy']['validate_only'] = False
            elif mutation == 'report_basename': value['candidate_basename'] = 'predecessor'
            else: value['report_source_sha256'] = '0' * 64
        _mutate_json(candidate, report_path, alter_report)
    else:
        def alter_closure(value):
            checks = value['checks']; first_critical = checks[0]; first_source = checks[35]
            if mutation == 'closure_legacy':
                value.clear(); value['failed_checks'] = 0
            elif mutation == 'closure_count': value['check_count'] = 991
            elif mutation == 'critical_missing': checks.pop(0)
            elif mutation == 'critical_duplicate': checks[1]['observation_key'] = first_critical['observation_key']
            elif mutation == 'source_digest':
                first_source['sha256'] = '0' * 64
                first_source['expected']['sha256'] = '0' * 64
                first_source['observed']['sha256'] = '0' * 64
            elif mutation == 'source_self': first_source['source_path'] = closure_path
            elif mutation == 'kind': first_source['kind'] = 'legacy'
            elif mutation == 'category': first_critical['category'] = 'raw_packages'
            elif mutation == 'passed_int': first_critical['passed'] = 1
            elif mutation == 'expected': first_critical['expected'] = 'candidate-claim'
            elif mutation == 'source_paths': first_critical['source_paths'] = []
            elif mutation == 'duplicate_check_id': checks[1]['check_id'] = first_critical['check_id']
            else: value['static_closure_sha256'] = '0' * 64
        _mutate_json(candidate, closure_path, alter_closure)
    expected_codes = {
        'report_missing': 'REPORT_SOURCE_SCHEMA',
        'report_unknown': 'REPORT_SOURCE_SCHEMA',
        'report_runtime': 'REPORT_RUNTIME',
        'report_plan': 'REPORT_PLAN',
        'report_focused': 'REPORT_FOCUSED',
        'report_attachment': 'REPORT_ATTACHMENTS',
        'report_static': 'CLOSURE_REPORT_COUNT_MISMATCH',
        'report_correction': 'REPORT_CORRECTION',
        'report_capsule': 'REPORT_CAPSULE',
        'report_basename': 'REPORT_CANDIDATE',
        'report_self_digest': 'REPORT_SOURCE_SCHEMA',
        'closure_legacy': 'STATIC_CLOSURE_SCHEMA',
        'closure_count': 'CLOSURE_REPORT_COUNT_MISMATCH',
        'critical_missing': 'STATIC_CLOSURE_CRITICAL',
        'critical_duplicate': 'STATIC_CLOSURE_CRITICAL',
        'source_digest': 'CLOSURE_PHYSICAL_SOURCE_MISMATCH',
        'source_self': 'CLOSURE_DEPENDENCY_CYCLE',
        'kind': 'STATIC_CLOSURE_KIND',
        'category': 'STATIC_CLOSURE_CRITICAL',
        'passed_int': 'STATIC_CLOSURE_RECORD',
        'expected': 'STATIC_CLOSURE_CRITICAL',
        'source_paths': 'STATIC_CLOSURE_CRITICAL',
        'duplicate_check_id': 'STATIC_CLOSURE_RECORD',
        'closure_self_digest': 'STATIC_CLOSURE_SCHEMA',
    }
    authority = _reseal(candidate)
    before = len(os.listdir('/proc/self/fd'))
    result = validate_frozen_candidate(candidate, expected_root_authority=authority, expected_runtime_identity=identity)
    _assert_single_failure(
        result, 'report_and_static_closure', expected_codes[mutation],
    )
    assert len(os.listdir('/proc/self/fd')) == before


def _late_mutation(candidate, mutation):
    correction = candidate / 'reports/correction_report.md'
    runtime = candidate / 'materialization/runtime_root/config/robot.yaml'
    if mutation == 'correction_rewrite':
        correction.chmod(0o644); correction.write_bytes(b'late correction mutation\n')
    elif mutation == 'runtime_rewrite':
        runtime.chmod(0o644); runtime.write_bytes(b'robot: late\n'); runtime.chmod(0o444)
    elif mutation == 'same_size':
        original = runtime.read_bytes(); changed = bytes([original[0] ^ 1]) + original[1:]
        runtime.chmod(0o644); runtime.write_bytes(changed); runtime.chmod(0o444)
    elif mutation == 'file_mode':
        correction.chmod(0o644)
    elif mutation == 'directory_mode':
        (candidate / 'focused_raw/case-001').chmod(0o755)
    elif mutation == 'extra_file':
        parent = candidate / 'reports'; parent.chmod(0o755)
        extra = parent / 'late-extra.txt'; extra.write_bytes(b'extra'); extra.chmod(0o444); parent.chmod(0o555)
    elif mutation == 'remove_file':
        parent = candidate / 'reports'; parent.chmod(0o755); correction.unlink(); parent.chmod(0o555)
    elif mutation == 'replace_inode':
        data = correction.read_bytes(); parent = correction.parent; parent.chmod(0o755)
        correction.unlink(); correction.write_bytes(data); correction.chmod(0o444); parent.chmod(0o555)
    elif mutation == 'replace_directory':
        parent = candidate / 'focused_raw'; target = parent / 'case-150'; parent.chmod(0o755)
        target.rename(parent / 'case-150.replaced'); target.mkdir(); target.chmod(0o555); parent.chmod(0o555)
    elif mutation == 'symlink_substitution':
        parent = correction.parent; parent.chmod(0o755); correction.unlink()
        os.symlink('../manifests/source_identity.json', correction); parent.chmod(0o555)
    elif mutation == 'hardlink_substitution':
        parent = correction.parent; parent.chmod(0o755); correction.unlink()
        os.link(candidate / 'manifests/source_identity.json', correction); parent.chmod(0o555)
    elif mutation == 'restored_bytes_changed_metadata':
        original = correction.read_bytes(); correction.chmod(0o644)
        correction.write_bytes(b'x' * len(original)); correction.write_bytes(original); correction.chmod(0o444)
    else:
        raise AssertionError(mutation)


@pytest.mark.parametrize(
    'mutation',
    ['correction_rewrite', 'runtime_rewrite', 'same_size', 'file_mode',
     'directory_mode', 'extra_file', 'remove_file', 'replace_inode',
     'replace_directory', 'symlink_substitution', 'hardlink_substitution',
     'restored_bytes_changed_metadata'],
    ids=lambda value: value,
)
def test_final_tree_barrier_rejects_late_mutation(tmp_path, monkeypatch, mutation):
    candidate, authority, identity = _fixture(tmp_path)
    original = candidate_module._validate_static_closure
    def mutate_after_closure(*args, **kwargs):
        result = original(*args, **kwargs)
        _late_mutation(candidate, mutation)
        return result
    monkeypatch.setattr(candidate_module, '_validate_static_closure', mutate_after_closure)
    before = len(os.listdir('/proc/self/fd'))
    result = validate_frozen_candidate(candidate, expected_root_authority=authority,
                                       expected_runtime_identity=identity)
    _assert_single_failure(result, 'candidate_inventory',
                           'CANDIDATE_CHANGED_AFTER_AUTHENTICATION')
    assert len(os.listdir('/proc/self/fd')) == before


def test_final_tree_barrier_passes_unchanged_candidate(tmp_path, monkeypatch):
    candidate, authority, identity = _fixture(tmp_path)
    calls = []
    original = candidate_module._CandidateReadSession.verify_current_tree
    def recording_barrier(self, runtime_root):
        calls.append(runtime_root)
        return original(self, runtime_root)
    monkeypatch.setattr(candidate_module._CandidateReadSession, 'verify_current_tree', recording_barrier)
    result = validate_frozen_candidate(candidate, expected_root_authority=authority,
                                       expected_runtime_identity=identity)
    assert result.overall == 'PASS'
    assert calls == ['materialization/runtime_root']
    assert len(result.traces) == 10 and all(trace.status == 'PASS' for trace in result.traces)


@pytest.mark.parametrize(
    ('stage', 'relative', 'mutation', 'code'),
    [
        ('six_plan_set', 'plans/production_root.json',
         lambda value: value.__setitem__('production_runtime_identity', 'f' * 64),
         'PLAN_RUNTIME_IDENTITY_MISMATCH'),
        ('focused_evidence', 'manifests/focused_results.json',
         lambda value: value['cases'][0].__setitem__('passed', False), 'FOCUSED_RECORD'),
        ('attachments', 'manifests/attachments.json',
         lambda value: value['attachments'].pop(), 'ATTACHMENT_SCHEMA'),
        ('report_and_static_closure', 'manifests/report_source.json',
         lambda value: value.__setitem__('unknown', True), 'REPORT_SOURCE_SCHEMA'),
        ('capsule_policy', 'manifests/capsule.json',
         lambda value: value.__setitem__('validate_only', False), 'CAPSULE_POLICY'),
    ],
    ids=['plans', 'focused', 'attachments', 'report', 'capsule'],
)
def test_each_stage_failure_has_exactly_one_fail_trace(tmp_path, stage, relative, mutation, code):
    candidate, _, identity = _fixture(tmp_path)
    _mutate_json(candidate, relative, mutation); authority = _reseal(candidate)
    result = validate_frozen_candidate(candidate, expected_root_authority=authority,
                                       expected_runtime_identity=identity)
    _assert_single_failure(result, stage, code)


def test_malformed_input_failure_has_exactly_one_trace():
    result = validate_frozen_candidate(object(), expected_root_authority={}, expected_runtime_identity=[])
    _assert_single_failure(result, 'candidate_inventory', 'DIGEST_INVALID')


@pytest.mark.parametrize(
    'field',
    ['focused_categories', 'focused_provenance', 'attachment_totals',
     'attachment_aggregates'],
)
def test_nested_fact_pair_aliases_are_detached(tmp_path, field):
    candidate, authority, identity = _fixture(tmp_path)
    facts = _upstream_facts(candidate, authority, identity)
    if field == 'focused_categories':
        source = [list(pair) for pair in facts.focused.category_totals]
        constructed = replace(facts.focused, category_totals=source)
        stored = constructed.category_totals
    elif field == 'focused_provenance':
        source = [list(pair) for pair in facts.focused.provenance_totals]
        constructed = replace(facts.focused, provenance_totals=source)
        stored = constructed.provenance_totals
    elif field == 'attachment_totals':
        source = [list(pair) for pair in facts.attachments.role_totals]
        constructed = replace(facts.attachments, role_totals=source)
        stored = constructed.role_totals
    else:
        source = [list(pair) for pair in facts.attachments.role_aggregates]
        constructed = replace(facts.attachments, role_aggregates=source)
        stored = constructed.role_aggregates
    snapshot = tuple(tuple(pair) for pair in source)
    source[0][1] = '0' * 64 if field == 'attachment_aggregates' else 999
    source.append(['unrelated', 1])
    assert stored == snapshot
    assert all(type(pair) is tuple for pair in stored)


@pytest.mark.parametrize('collection', ['traces', 'counts', 'side_effects'])
def test_exported_result_collection_aliases_are_detached(collection):
    traces = [candidate_module.ValidationTrace(name, 'PASS', 'OK')
              for name in candidate_module.TRACE_NAMES]
    counts = [list(pair) for pair in candidate_module._RESULT_COUNTS_PASS.items()]
    side_effects = dict(candidate_module._SIDE_EFFECTS)
    result = candidate_module.CandidateValidationResult(
        candidate_module.RESULT_SCHEMA, 'PASS', '/synthetic', '0' * 64,
        '1' * 64, traces, counts, side_effects,
    )
    expected_traces = result.traces
    expected_counts = result.counts
    expected_side_effects = result.side_effects
    if collection == 'traces':
        traces.pop()
    elif collection == 'counts':
        counts[0][1] = 0
        counts.append(['invalid', 1])
    else:
        side_effects['process_factory_calls'] = 1
    assert result.traces == expected_traces
    assert result.counts == expected_counts
    assert result.side_effects == expected_side_effects
    assert type(result.traces) is tuple
    assert all(type(pair) is tuple for pair in result.counts + result.side_effects)


@pytest.mark.parametrize(
    ('field', 'value'),
    [('name', 'invalid'), ('status', 'NOT_RUN'), ('code', ''), ('detail', [])],
)
def test_exported_trace_rejects_invalid_fields(field, value):
    arguments = {
        'name': 'candidate_inventory', 'status': 'PASS', 'code': 'OK',
        'detail': '',
    }
    arguments[field] = value
    with pytest.raises(candidate_module.CandidateValidateOnlyError):
        candidate_module.ValidationTrace(**arguments)


def _construct_result(overall, traces, *, counts=None, side_effects=None):
    expected_counts = (
        candidate_module._RESULT_COUNTS_PASS
        if overall == 'PASS' else candidate_module._RESULT_COUNTS_FAIL
    )
    return candidate_module.CandidateValidationResult(
        candidate_module.RESULT_SCHEMA, overall, '/synthetic', '0' * 64,
        '1' * 64, traces,
        dict(expected_counts) if counts is None else counts,
        dict(candidate_module._SIDE_EFFECTS) if side_effects is None else side_effects,
    )


@pytest.mark.parametrize(
    'contract',
    ['incomplete_pass', 'two_failures', 'invalid_order', 'duplicate_name',
     'missing_prefix'],
)
def test_exported_result_rejects_invalid_trace_contract(contract):
    passing = [candidate_module.ValidationTrace(name, 'PASS', 'OK')
               for name in candidate_module.TRACE_NAMES]
    if contract == 'incomplete_pass':
        overall, traces = 'PASS', passing[:-1]
    elif contract == 'two_failures':
        overall = 'FAIL'
        traces = [
            candidate_module.ValidationTrace('candidate_inventory', 'FAIL', 'FIRST'),
            candidate_module.ValidationTrace('root_authority', 'FAIL', 'SECOND'),
        ]
    elif contract == 'invalid_order':
        overall = 'FAIL'
        traces = [
            candidate_module.ValidationTrace('root_authority', 'PASS', 'OK'),
            candidate_module.ValidationTrace('six_plan_set', 'FAIL', 'BROKEN'),
        ]
    elif contract == 'duplicate_name':
        overall = 'FAIL'
        traces = [
            candidate_module.ValidationTrace('candidate_inventory', 'PASS', 'OK'),
            candidate_module.ValidationTrace('candidate_inventory', 'FAIL', 'BROKEN'),
        ]
    else:
        overall = 'FAIL'
        traces = [
            candidate_module.ValidationTrace('root_authority', 'FAIL', 'BROKEN'),
        ]
    with pytest.raises(candidate_module.CandidateValidateOnlyError):
        _construct_result(overall, traces)


def test_exported_result_rejects_boolean_count():
    traces = [candidate_module.ValidationTrace(name, 'PASS', 'OK')
              for name in candidate_module.TRACE_NAMES]
    counts = dict(candidate_module._RESULT_COUNTS_PASS)
    counts['runtime_members'] = True
    with pytest.raises(candidate_module.CandidateValidateOnlyError):
        _construct_result('PASS', traces, counts=counts)


def test_exported_result_rejects_nested_side_effect_value():
    traces = [candidate_module.ValidationTrace(name, 'PASS', 'OK')
              for name in candidate_module.TRACE_NAMES]
    side_effects = dict(candidate_module._SIDE_EFFECTS)
    side_effects['process_factory_calls'] = []
    with pytest.raises(candidate_module.CandidateValidateOnlyError):
        _construct_result('PASS', traces, side_effects=side_effects)


@pytest.mark.parametrize('fact_name', ['bundle', 'capsule', 'invocation'])
def test_all_pair_facts_reject_duplicate_keys(tmp_path, fact_name):
    candidate, authority, identity = _fixture(tmp_path)
    facts = _upstream_facts(candidate, authority, identity)
    with pytest.raises(candidate_module.CandidateValidateOnlyError):
        if fact_name == 'bundle':
            pairs = list(facts.bundle.paths)
            pairs.append(('inventory_path', 'manifests/duplicate-inventory.json'))
            replace(facts.bundle, paths=pairs)
        elif fact_name == 'capsule':
            pairs = list(facts.capsule.policy)
            pairs.append(('validate_only', True))
            replace(facts.capsule, policy=pairs)
        else:
            pairs = list(facts.invocation.side_effects)
            pairs.append(('process_factory_calls', 0))
            replace(facts.invocation, side_effects=pairs)
