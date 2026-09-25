import math
import unittest
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from minimal_pipeline import load_inputs, q1, q2
from final_joint import strict_q3
from continuous_joint import build_model
from q3_official_semantics import (
    component_charge_time, dynamic_communication_audit, hard_deadline_ok,
    relay_component_ledger, relay_leg_time_energy, relay_time_chain, resource_peak,
)


class Q3OfficialSemanticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = load_inputs()
        _, batches = q1(cls.data)
        cls.tasks, _ = q2(cls.data, batches)
        cls.cert = strict_q3(cls.data, cls.tasks)
        cls.task = cls.tasks.iloc[0]
        cls.cert_row = cls.cert.iloc[0].to_dict()

    def test_hard_deadline_is_hard(self):
        self.assertTrue(hard_deadline_ok(100.0, 100.0))
        self.assertFalse(hard_deadline_ok(100.000002, 100.0))

    def test_build_model_contains_non_relaxable_hard_deadline(self):
        _, _, bounds, cons, names = build_model(self.data, self.tasks.iloc[:1].copy(), self.cert.iloc[:1].copy(), 2, objective='zero')
        hard_vars = [i for i, n in enumerate(names) if n.startswith('tardy_box_')]
        self.assertTrue(hard_vars)
        # The representative task contains contractual hard cargo; its tardy
        # variable is fixed to zero and therefore cannot relax delivery <= D.
        self.assertTrue(all(float(bounds.ub[i]) == 0.0 for i in hard_vars))

    def test_relay_flight_time_formula(self):
        r = self.data['relay']
        t, e = relay_leg_time_energy(self.data, 1500.0, 200.0, 100.0, 130.0)
        expected_t = (150.0 / r['climb_speed']) + 1500.0 / r['speed'] + (250.0 - 130.0) / r['descend_speed']
        expected_e = r['power_kw'] * 1500.0 / r['speed'] / 3600.0 + r['mass'] * 9.81 * 150.0 / (3.6e6 * r['eta_climb'])
        self.assertAlmostEqual(t, expected_t, places=9)
        self.assertAlmostEqual(e, expected_e, places=9)

    def test_relay_service_energy(self):
        chain = relay_time_chain(self.data, str(self.task['service']), self.cert_row, 120.0)
        expected = (self.data['relay']['hover_power_kw'] + self.data['relay']['comm_power_kw']) * 120.0 / 3600.0
        self.assertAlmostEqual(chain['service_energy_kwh'], expected, places=9)
        self.assertGreater(chain['total_energy_kwh'], chain['service_energy_kwh'])

    def test_relay_energy_soc(self):
        led = relay_component_ledger(self.data, 'R-E01', 0.0, 100.0, 0.5)
        self.assertAlmostEqual(led['soc_end'], 1.0 - 0.5 / self.data['relay']['energy_kwh'])
        self.assertTrue(led['reserve_ok'])

    def test_relay_energy_charging(self):
        soc = 0.8
        expected = self.data['relay']['component_full_charge_s'] * (0.65 * (0.90 - soc) / 0.90 + 0.35)
        self.assertAlmostEqual(component_charge_time(soc, self.data['relay']['component_full_charge_s']), expected)
        self.assertAlmostEqual(component_charge_time(0.95, 1800.0), 1800.0 * 0.35 * 0.05 / 0.10)

    def test_relay_inventory(self):
        events = [{'resource': 'relay_energy_component', 'start_s': 0, 'end_s': 10},
                  {'resource': 'relay_energy_component', 'start_s': 10, 'end_s': 20}]
        peak, ok = resource_peak(events, 'relay_energy_component', self.data['relay']['component_inventory'])
        self.assertEqual(peak, 1)
        self.assertTrue(ok)
        self.assertEqual(self.data['relay']['component_inventory'], 6)

    def test_dynamic_communication(self):
        # A degenerate but valid direct interval at O01 verifies the state
        # machine chooses direct service and does not require a relay.
        o = self.data['center']; h = float(o['海拔（m）']) + 50.0
        traj = [{'t0': 0.0, 't1': 1.0, 'a': o, 'b': o, 'ha': h, 'hb': h, 'phase': 'cruise'}]
        rows = dynamic_communication_audit(self.data, str(self.task['service']), str(self.task['type']), self.cert_row, traj)
        self.assertEqual(len(rows), 1)
        self.assertTrue(rows[0]['direct_available'])
        self.assertFalse(rows[0]['relay_required'])
        self.assertTrue(rows[0]['communication_ok'])

    def test_direct_interval_no_relay(self):
        o = self.data['center']; h = float(o['海拔（m）']) + 50.0
        traj = [{'t0': 0.0, 't1': 1.0, 'a': o, 'b': o, 'ha': h, 'hb': h}]
        rows = dynamic_communication_audit(self.data, str(self.task['service']), str(self.task['type']), self.cert_row, traj)
        self.assertEqual(rows[0]['relay_id'], '')
        self.assertFalse(rows[0]['relay_required'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
