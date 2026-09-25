import pandas as pd
def test_relay_energy_parity():
    d=pd.read_csv("logs/parity_task_diff.csv"); q=d[d.field=="relay_energy"]; assert (q.delta.abs()<=1e-8).all()
