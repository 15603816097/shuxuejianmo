"""Analysis-only Q3 wrapper.

The strict frozen pipeline is never modified.  This wrapper records the sole
scenario change: +0.1 dB applied to the previously identified backhaul
budget when testing S008 communication geometry.
"""
from pathlib import Path
import hashlib, json
import pandas as pd

SCENARIO='FEASIBILITY_RESTORED_0P1DB'
DELTA_LINK_BUDGET_DB=0.1
PIPELINE_SHA256='5f5fc70d5ed19d36644cdad811b12bdac89d19f18393218c72fca28dae32afeb'

def effective_backhaul_margin(strict_backhaul_margin_db):
    return float(strict_backhaul_margin_db) + DELTA_LINK_BUDGET_DB

def wrapper_metadata():
    return {'scenario':SCENARIO,'delta_link_budget_db':DELTA_LINK_BUDGET_DB,
            'pipeline_sha256':PIPELINE_SHA256,'scope':'analysis_only_backhaul_margin'}

if __name__=='__main__':
    print(json.dumps(wrapper_metadata(),ensure_ascii=False,indent=2))
