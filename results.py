# import numpy as np
#
# # NPZ 파일에서 데이터 로드
#
# data = np.load('experiments/kitchen/saved_runs/run_tamp/2025-08-31-12-10-30/easy_box_small_basket_backtrack_True/3_1_10_1/result.npz', allow_pickle=True)
#
# # 키 확인하기
#
# keys = data.files
#
# print(keys)
# print(data[keys[0]])
# print(data[keys[1]])
# print(data[keys[2]])
# print(data[keys[3]])
# print(data[keys[4]])

import pandas as pd
import numpy as np
import re
from distutils.util import strtobool

csv_path = "experiments/kitchen/summary_old.csv"
df = pd.read_csv(csv_path)

# 1) prob_num 숫자화
df["prob_num"] = pd.to_numeric(df["prob_num"], errors="coerce").astype("Int64")

# 2) sim_success → bool
def to_bool(x):
    if pd.isna(x):
        return False
    if isinstance(x, (bool, np.bool_)):
        return bool(x)
    s = str(x).strip().strip("'\"").lower()
    s = re.sub(r'^[^\w]+', '', s)  # 앞쪽 특수문자 제거(뷰어 표기 대응)
    try:
        return bool(strtobool(s))
    except Exception:
        return False

df["sim_success_bool"] = df["success"].map(to_bool)

# 3) total_planning_time → float(초)
def to_seconds(x):
    if pd.isna(x):
        return np.nan
    if isinstance(x, (int, float, np.number)):
        return float(x)
    s = str(x).strip()

    # HH:MM:SS(.ms) 또는 MM:SS(.ms)
    m = re.match(r'^(?:(\d+):)?(\d{1,2}):(\d{1,2}(?:\.\d+)?)$', s)
    if m:
        h = float(m.group(1) or 0)
        mi = float(m.group(2))
        se = float(m.group(3))
        return h*3600 + mi*60 + se

    # "1h 2m 3.4s" / "2m 10s" / "12s"
    m = re.match(r'^(?:(\d+(?:\.\d+)?)\s*h)?\s*(?:(\d+(?:\.\d+)?)\s*m)?\s*(?:(\d+(?:\.\d+)?)\s*s)?$', s)
    if m and any(m.group(i) for i in range(1,4)):
        h = float(m.group(1) or 0)
        mi = float(m.group(2) or 0)
        se = float(m.group(3) or 0)
        return h*3600 + mi*60 + se

    # 숫자만 추출 (쉼표 제거 포함)
    nums = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', s.replace(',', ''))
    return float(nums[0]) if nums else np.nan

df["total_planning_time_sec"] = df["total_planning_time"].map(to_seconds) # planning_time_sec for pddlstream

# 4) 대상 prob_num
targets = [3, 4, 5, 6]
df_t = df[df["prob_num"].isin(targets)].copy()

# 4-1) 평균 시간(성공한 케이스만)
avg_time = (
    df_t[df_t["sim_success_bool"]]
      .groupby("prob_num", as_index=False)["total_planning_time_sec"]
      .mean()
      .rename(columns={"total_planning_time_sec": "avg_total_planning_time_sec"})
)

# 4-2) 성공률(성공 개수 / 전체 개수)
succ = (
    df_t.groupby("prob_num")
        .agg(success_count=("sim_success_bool", "sum"),
             total_count=("sim_success_bool", "size"))
        .reset_index()
)
succ["success_rate"] = succ["success_count"] / succ["total_count"]

# 4-3) num_llm_calls 평균 (0 제외)
num_llm_calls = (
    df_t[df_t["num_llm_calls"] > 0]   # 0이 아닌 경우만 필터링
      .groupby("prob_num")
      .agg(avg_num_llm_calls=("num_llm_calls", "mean"))
      .reset_index()
)

# 5) 합치고 targets 순서로 정렬, 퍼센트 컬럼 추가
out = (
    succ.merge(avg_time, on="prob_num", how="left")
        .merge(num_llm_calls, on="prob_num", how="left")
        .set_index("prob_num")
        .reindex(targets)
        .reset_index()
)
out["success_rate_pct"] = (out["success_rate"] * 100).round(2)


# 보기 좋게 출력
print(out[[
    "prob_num",
    "success_count",
    "total_count",
    "success_rate",
    "success_rate_pct",
    "avg_total_planning_time_sec",
    "avg_num_llm_calls"
]].to_string(index=False))
# 필요하면 파일로 저장
# out.to_csv("/home/minseo/robot_ws/src/tamp_llm/experiments/blocksworld_pr/ours_substep2/batch_outputs/summary_by_probnum.csv", index=False)
