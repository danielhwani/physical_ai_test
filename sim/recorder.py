"""시험 데이터 기록 (문서 §4: Parquet, §9.2: Live/Virtual 출처 표시, §13.3: 버전 조합 기록)."""
import json
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq


class Recorder:
    def __init__(self, out_dir, run_meta):
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.meta = run_meta
        self.rows = {}

    def log(self, **fields):
        for k, v in fields.items():
            v = np.asarray(v)
            if v.ndim == 0:
                self.rows.setdefault(k, []).append(v.item())
            else:
                for i, vi in enumerate(v.ravel()):
                    self.rows.setdefault(f"{k}_{i}", []).append(float(vi))

    def save(self, mop):
        table = pa.table(self.rows)
        meta = {b"run_meta": json.dumps(self.meta, ensure_ascii=False).encode()}
        table = table.replace_schema_metadata(meta)
        pq.write_table(table, self.out_dir / "timeseries.parquet")
        (self.out_dir / "summary.json").write_text(
            json.dumps({"meta": self.meta, "mop": mop}, indent=2, ensure_ascii=False))
