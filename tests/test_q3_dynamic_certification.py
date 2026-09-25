import json
from pathlib import Path
import unittest
import pandas as pd
ROOT=Path(__file__).resolve().parents[1]

class DynamicCertificationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.summary=json.loads((ROOT/'logs/q3_Q2_001_dynamic_timeline.md').read_text())
        cls.cert=pd.read_csv(ROOT/'results/q3_Q2_001_continuous_certification.csv')
        cls.timeline=pd.read_csv(ROOT/'results/q3_Q2_001_timeline_continuity_audit.csv')
        cls.trace=pd.read_csv(ROOT/'results/q3_runtime_parameter_trace.csv')
    def test_continuous_direct_certification(self):
        self.assertEqual(len(self.cert),549)
        self.assertTrue(self.cert.continuous_certified.all())
        self.assertTrue((self.cert.margin_lower_bound >= -1e-6).all())
    def test_timeline_no_gap(self):
        self.assertTrue(self.timeline['gap'].abs().le(1e-6).all())
        self.assertTrue(self.timeline['pass'].all())
    def test_timeline_no_overlap(self):
        self.assertTrue(self.timeline['overlap'].le(1e-6).all())
        self.assertTrue(self.timeline['duration'].ge(-1e-6).all())
    def test_runtime_parameters_from_files(self):
        self.assertEqual(int(self.trace.hardcoded_in_runtime_path.astype(str).str.lower().eq('true').sum()),0)
        self.assertTrue(self.trace.loaded_from_attachment.astype(str).str.lower().eq('true').all())
    def test_zero_relay_required_is_valid(self):
        self.assertEqual(int(self.summary['relay_required_intervals'].__len__()),0)
        self.assertTrue(self.summary['communication_pass'])
        self.assertIsNone(self.summary['relay_chain'])

if __name__=='__main__': unittest.main(verbosity=2)
