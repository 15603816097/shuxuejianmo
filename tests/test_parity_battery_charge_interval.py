import pandas as pd
def test_battery_charge_interval_parity():
    d=pd.read_csv("logs/parity_resource_diff.csv"); q=d[d.resource.str.startswith("battery:")]; assert (q[["delta_start","delta_end"]].abs()<=1e-8).all().all()
