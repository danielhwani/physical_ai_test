#!/usr/bin/env bash
# MuJoCo Menagerie의 Unitree Go2 모델을 고정 커밋으로 받는다 (specs/go2_control.yaml의 menagerie_commit).
set -euo pipefail
cd "$(dirname "$0")/.."
COMMIT=4d038b3feae26ec82b46a4d586379114012a8ac7
DEST=third_party/mujoco_menagerie
if [ ! -d "$DEST/.git" ]; then
  mkdir -p third_party
  git clone --filter=blob:none --sparse https://github.com/google-deepmind/mujoco_menagerie "$DEST"
fi
git -C "$DEST" sparse-checkout set unitree_go2
git -C "$DEST" fetch --depth 1 origin "$COMMIT" 2>/dev/null || git -C "$DEST" fetch origin
git -C "$DEST" -c advice.detachedHead=false checkout "$COMMIT"
echo "OK: $DEST/unitree_go2 @ ${COMMIT:0:7}"
