import pandas as pd
def test_deadline_parity():
    d=pd.read_csv("logs/parity_delivery_diff.csv"); q=d[d.field=="late_flag"]
    assert len(q)==80 and (q.delta.abs()<=1e-8).all()
