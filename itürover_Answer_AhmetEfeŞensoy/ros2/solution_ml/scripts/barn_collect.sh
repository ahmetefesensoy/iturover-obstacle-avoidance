#!/bin/bash
# barn_collect.sh — TEB expert data collection across BARN worlds.
# Time-based: collects for RUN_DURATION seconds, keeps if robot moved enough.
# Goal success is a bonus (faster shutdown) but NOT required to save data.


source /opt/ros/noetic/setup.bash
source /root/rover_ws/devel/setup.bash

BARN_DIR="/root/BARN_dataset/BARN_dataset"
OUT_DIR="$HOME/rover_ml_data"
N_WORLDS="${1:-150}"
N_RUNS="${2:-2}"
RUN_DURATION=180          # her run için süre
MIN_KB=30                 # bu kadardan küçük dosyalar atılır (robot hiç hareket etmemişse)
COOLDOWN=8
MIN_FREE_MB=800
MIN_FREE_RAM_MB=400
MAX_CPU_PCT=85
MAX_WAIT_CYCLES=6
SUCCESS_MARKER="/tmp/barn_goal_reached"
STATE_FILE="$HOME/barn_collect_state.txt"
LOG_FILE="$HOME/barn_collect.log"

mkdir -p "$OUT_DIR"

log() { echo "[$(date '+%H:%M:%S')] $*" | tee -a "$LOG_FILE"; }

# ── World listesi ─────────────────────────────────────────────────────────────
WORLDS=()
STEP=$(( 300 / N_WORLDS ))
for i in $(seq 0 $STEP 299); do
    WORLDS+=($i)
    [ ${#WORLDS[@]} -ge $N_WORLDS ] && break
done
TOTAL=${#WORLDS[@]}

# ── Resume ────────────────────────────────────────────────────────────────────
START_IDX=0
RESUME_SUCC=0
if [ -f "$STATE_FILE" ]; then
    LAST_WORLD=$(grep "^LAST="  "$STATE_FILE" | cut -d= -f2)
    SAVED_N=$(   grep "^TOTAL=" "$STATE_FILE" | cut -d= -f2)
    LAST_SUCC=$( grep "^SUCC="  "$STATE_FILE" | cut -d= -f2)
    if [ -n "$LAST_WORLD" ] && [ "$SAVED_N" = "$TOTAL" ]; then
        for i in "${!WORLDS[@]}"; do
            if [ "${WORLDS[$i]}" = "$LAST_WORLD" ]; then
                if [ "${LAST_SUCC:-0}" -ge "$N_RUNS" ]; then
                    START_IDX=$(( i + 1 )); RESUME_SUCC=0
                else
                    START_IDX=$i; RESUME_SUCC=${LAST_SUCC:-0}
                fi
                break
            fi
        done
        if [ $START_IDX -ge $TOTAL ]; then
            log "Tüm $TOTAL world tamamlandı. Sıfırlamak için: rm $STATE_FILE"
            exit 0
        fi
        log "RESUME: world ${WORLDS[$START_IDX]} succ=$RESUME_SUCC/$N_RUNS"
    else
        rm -f "$STATE_FILE"
    fi
fi

EST_MIN=$(( (TOTAL - START_IDX) * N_RUNS * (RUN_DURATION + COOLDOWN + 40) / 60 ))
log "=============================="
log " BARN Data Collection (TEB expert, time-based)"
log " ${TOTAL} world × ${N_RUNS} run"
log " Run süresi: ${RUN_DURATION}s  Min boyut: ${MIN_KB}KB"
log " Tahmini süre: ~${EST_MIN} dk"
log "=============================="

save_state() { printf "LAST=%s\nTOTAL=%s\nSUCC=%s\n" "$1" "$TOTAL" "$2" > "$STATE_FILE"; }

check_disk() {
    FREE_MB=$(df "$OUT_DIR" | awk 'NR==2 {printf "%d", $4/1024}')
    if [ "$FREE_MB" -lt "$MIN_FREE_MB" ]; then
        log "SAFETY STOP ► Disk doldu (${FREE_MB}MB < ${MIN_FREE_MB}MB)"
        exit 1
    fi
}

wait_for_ram() {
    local cycles=0
    while true; do
        FREE_RAM=$(free -m | awk '/^Mem:/ {print $7}')
        [ "$FREE_RAM" -ge "$MIN_FREE_RAM_MB" ] && return 0
        cycles=$(( cycles + 1 ))
        [ $cycles -ge $MAX_WAIT_CYCLES ] && { log "SAFETY STOP ► RAM düzelmedi"; exit 1; }
        log "  RAM düşük (${FREE_RAM}MB) — 10sn bekleniyor"
        sleep 10
    done
}

get_cpu_pct() {
    local C1 C2
    C1=($(head -1 /proc/stat))
    sleep 1
    C2=($(head -1 /proc/stat))
    local i1=${C1[4]} t1=$(( ${C1[1]}+${C1[2]}+${C1[3]}+${C1[4]}+${C1[5]}+${C1[6]}+${C1[7]} ))
    local i2=${C2[4]} t2=$(( ${C2[1]}+${C2[2]}+${C2[3]}+${C2[4]}+${C2[5]}+${C2[6]}+${C2[7]} ))
    local dt=$(( t2-t1 )); [ $dt -eq 0 ] && echo 0 && return
    echo $(( (1000*(dt-(i2-i1))/dt+5)/10 ))
}

wait_for_cpu() {
    local cycles=0
    while true; do
        local pct; pct=$(get_cpu_pct)
        [ "$pct" -le "$MAX_CPU_PCT" ] && return 0
        cycles=$(( cycles + 1 ))
        [ $cycles -ge $MAX_WAIT_CYCLES ] && { log "SAFETY STOP ► CPU %${pct}"; exit 1; }
        log "  CPU yüksek (%${pct}) — bekleniyor"
        sleep 10
    done
}

kill_ros() {
    pkill -f "gzserver"           2>/dev/null || true
    pkill -f "gzclient"           2>/dev/null || true
    pkill -f "move_base"          2>/dev/null || true
    pkill -f "rosmaster"          2>/dev/null || true
    pkill -f "barn_goal_publisher"2>/dev/null || true
    pkill -f "odom_to_tf"         2>/dev/null || true
    pkill -f "collect_data"       2>/dev/null || true
    pkill -f "shutdown_timer"     2>/dev/null || true
    sleep 2
    pkill -9 -f "gzserver"        2>/dev/null || true
    sleep 5
}

# ── Ana döngü ─────────────────────────────────────────────────────────────────
for idx in $(seq $START_IDX $(( TOTAL - 1 ))); do
    WORLD_IDX=${WORLDS[$idx]}
    WORLD_FILE="$BARN_DIR/world_files/world_$WORLD_IDX.world"
    WORLD_TAG="barn_$WORLD_IDX"

    log ""
    log "[$((idx+1))/$TOTAL] world_$WORLD_IDX"

    if [ ! -f "$WORLD_FILE" ]; then
        log "  ATLA: world dosyası yok"
        save_state "$WORLD_IDX" "$N_RUNS"
        continue
    fi

    SPAWN=$(python3 - <<EOF
import numpy as np, os
pf = "$BARN_DIR/path_files/path_$WORLD_IDX.npy"
if os.path.exists(pf):
    p = np.load(pf)
    ox, oy, r = -4.5, 0.0, 0.15
    sx = ox + p[0][1]*r + r/2
    sy = oy + p[0][0]*r + r/2
    print(f"{sx:.3f} {sy:.3f}")
else:
    print("-4.42 3.98")
EOF
)
    SPAWN_X=$(echo $SPAWN | cut -d' ' -f1)
    SPAWN_Y=$(echo $SPAWN | cut -d' ' -f2)

    successes=$RESUME_SUCC
    RESUME_SUCC=0

    while [ $successes -lt $N_RUNS ]; do
        run_num=$(( successes + 1 ))
        OUT_PATH="$OUT_DIR/${WORLD_TAG}_r${run_num}_$(date +%Y%m%d_%H%M%S).npz"

        check_disk
        wait_for_ram
        wait_for_cpu
        kill_ros
        rm -f "$SUCCESS_MARKER"

        log "  run $run_num/$N_RUNS  spawn=($SPAWN_X,$SPAWN_Y)"

        timeout $(( RUN_DURATION + 60 )) \
            roslaunch reactive_robot barn_nav_collect.launch \
                world_file:="$WORLD_FILE" \
                world_idx:=$WORLD_IDX \
                world_tag:="$WORLD_TAG" \
                run_duration:=$RUN_DURATION \
                out_path:="$OUT_PATH" \
                spawn_x:=$SPAWN_X \
                spawn_y:=$SPAWN_Y \
            2>/dev/null || true

        if [ -f "$OUT_PATH" ]; then
            SIZE_KB=$(du -k "$OUT_PATH" | cut -f1)
            if [ "$SIZE_KB" -ge "$MIN_KB" ]; then
                GOAL_TAG=""
                [ -f "$SUCCESS_MARKER" ] && GOAL_TAG=" ✓hedefe ulaştı"
                log "  ✓ run $run_num — ${SIZE_KB}KB kaydedildi${GOAL_TAG}"
                successes=$(( successes + 1 ))
                save_state "$WORLD_IDX" "$successes"
            else
                log "  ✗ run $run_num — çok küçük (${SIZE_KB}KB < ${MIN_KB}KB), robot hareket etmedi"
                rm -f "$OUT_PATH"
            fi
        else
            log "  ✗ run $run_num — dosya oluşmadı"
        fi

        [ $successes -lt $N_RUNS ] && sleep $COOLDOWN
    done

    save_state "$WORLD_IDX" "$N_RUNS"
done

rm -f "$STATE_FILE"
SAVED=$(ls "$OUT_DIR"/barn_*.npz 2>/dev/null | wc -l)
log ""
log "=============================="
log "Tamamlandi! $SAVED dosya → $OUT_DIR"
log "=============================="
