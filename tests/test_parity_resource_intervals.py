import pandas as pd
def test_transport_interval_parity():
    d=pd.read_csv("logs/parity_resource_diff.csv"); q=d[d.resource.str.startswith("transport:")]; assert (q[["delta_start","delta_end"]].abs()<=1e-8).all().all()
