#!/usr/bin/env python3
"""Check the configured owners and topics of the navigation velocity link."""

import re
from pathlib import Path

import yaml

MODULE_ROOT = Path(__file__).resolve().parents[1]
ROOT = next(parent for parent in MODULE_ROOT.parents
            if (parent / 'Modules/modules.yaml').is_file())


def main():
    config = yaml.safe_load((ROOT / 'User/RobotConfig/sentry_gimbal.yaml').read_text())
    modules = {m['id']: m for m in config['modules']}

    def args(module_id):
        return {key: value for arg in modules[module_id]['args'] for key, value in arg.items()}

    def topics(module_id):
        return [re.search(r'\{"([^"]+)"\}', value).group(1)
                for value in args(module_id)['topic_configs']]

    assert modules['host_link']['module'] == 'GUET-PLDX/NavLink'
    assert args('host_link')['cmd'] == 'cmd'
    assert 'uart' not in args('host_link')
    for module in modules.values():
        values = {key: value for arg in module.get('args', []) for key, value in arg.items()}
        if values.get('uart') == 'usb_navigation_cdc':
            assert module['module'] in {
                'GUET-PLDX/SharedTopic', 'GUET-PLDX/SharedTopicClient',
            }
    assert args('navigation_rx')['uart'] == args('navigation_tx')['uart'] == 'usb_navigation_cdc'
    assert topics('navigation_rx') == ['nav_data', 'behavior_data']
    assert topics('navigation_tx') == [
        'team_info', 'game_info', 'online_info', 'offline_info', 'radar_info',
        'chassis_feedback',
    ]
    assert args('navigation_tx')['slot_count'] > 0
    assert modules['talos_link']['module'] == 'GUET-PLDX/TalosLink'
    assert args('talos_link')['cmd'] == 'cmd'
    assert args('talos_link')['uart'] == 'usb_vision_cdc'
    assert all(m['module'] != 'GUET-PLDX/HostData' for m in modules.values())
    registry = yaml.safe_load((ROOT / 'Modules/modules.yaml').read_text())['modules']
    assert 'GUET-PLDX/NavLink' in [r.split('@')[0] if isinstance(r, str) else r['id'] for r in registry]
    source = (MODULE_ROOT / 'HostDataLink.hpp').read_text()
    assert 'navigation_velocity' not in source
    for field in ('vx_mps', 'vy_mps', 'wz_rad_s'):
        assert f'command.{field}' in source
    assert 'MakeAIChassis(latest_chassis_' in source
    assert 'value.chassis.ff_valid = true' not in source
    assert 'chassis_ff_topic_' not in source
    assert source.index('tracker_.Observe(value, NOW)') < source.index('latest_chassis_ = value')
    print('PASS: navigation velocity ownership and Talos visual link configuration')


if __name__ == '__main__':
    main()
