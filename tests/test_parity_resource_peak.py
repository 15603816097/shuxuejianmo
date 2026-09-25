import pandas as pd
def test_resource_peak_parity():
    d=pd.read_csv("logs/parity_resource_diff.csv"); q=d[d.task_id=="__PEAK__"]; assert (q.delta_start.abs()<=1e-8).all()
