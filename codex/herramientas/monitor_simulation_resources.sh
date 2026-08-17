#!/usr/bin/env bash
set -uo pipefail

PGID=""
CSV_FILE=""
SUMMARY_FILE=""
STOP_FILE=""
GUARD_FILE=""
SAMPLE_SEC="1"
MIN_AVAILABLE_MIB="1024"
MAX_MEMORY_PSI_FULL_AVG10="20"
GUARD_CONSECUTIVE="3"

usage() {
  cat <<USAGE
Uso: $0 --pgid N --csv FILE --summary FILE --stop-file FILE --guard-file FILE [opciones]

Opciones:
  --sample-sec SEC
  --min-available-mib MIB
  --max-memory-psi-full-avg10 PCT
  --guard-consecutive N
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --pgid) PGID="${2:-}"; shift 2 ;;
    --csv) CSV_FILE="${2:-}"; shift 2 ;;
    --summary) SUMMARY_FILE="${2:-}"; shift 2 ;;
    --stop-file) STOP_FILE="${2:-}"; shift 2 ;;
    --guard-file) GUARD_FILE="${2:-}"; shift 2 ;;
    --sample-sec) SAMPLE_SEC="${2:-}"; shift 2 ;;
    --min-available-mib) MIN_AVAILABLE_MIB="${2:-}"; shift 2 ;;
    --max-memory-psi-full-avg10) MAX_MEMORY_PSI_FULL_AVG10="${2:-}"; shift 2 ;;
    --guard-consecutive) GUARD_CONSECUTIVE="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Argumento desconocido: $1" >&2; usage; exit 2 ;;
  esac
done

if [ -z "$PGID" ] || [ -z "$CSV_FILE" ] || [ -z "$SUMMARY_FILE" ] ||
   [ -z "$STOP_FILE" ] || [ -z "$GUARD_FILE" ]; then
  usage
  exit 2
fi

mkdir -p "$(dirname "$CSV_FILE")" "$(dirname "$SUMMARY_FILE")"
rm -f "$STOP_FILE" "$GUARD_FILE"

read_meminfo_kib() {
  local key="$1"
  awk -v key="$key" '$1 == key":" {print $2; exit}' /proc/meminfo
}

read_pressure_avg10() {
  local file="$1"
  local kind="$2"
  awk -v kind="$kind" '
    $1 == kind {
      for (i = 2; i <= NF; ++i) {
        if ($i ~ /^avg10=/) {
          split($i, value, "=");
          print value[2];
          exit;
        }
      }
    }
  ' "$file" 2>/dev/null
}

read_vmstat_value() {
  local key="$1"
  awk -v key="$key" '$1 == key {print $2; exit}' /proc/vmstat
}

read_cpu_counters() {
  awk '/^cpu / {print $2, $3, $4, $5, $6, $7, $8, $9; exit}' /proc/stat
}

summarize_process_group() {
  ps -eo pgid=,rss=,pcpu=,comm=,args= --no-headers 2>/dev/null | awk -v pgid="$PGID" '
    BEGIN {
      total_rss = total_cpu = count = server = orb = gazebo = rviz = web = 0;
      max_rss = 0;
      max_comm = "none";
      external_max_rss = 0;
      external_max_comm = "none";
    }
    ($1 + 0) == (pgid + 0) {
      rss = $2 + 0;
      cpu = $3 + 0;
      comm = $4;
      total_rss += rss;
      total_cpu += cpu;
      count += 1;
      if (rss > max_rss) {
        max_rss = rss;
        max_comm = comm;
      }
      line = tolower($0);
      comm_lower = tolower(comm);
      if (comm_lower ~ /global_map_serv/ || line ~ /\/global_map_server([[:space:]]|$)/) server += rss;
      if (comm_lower ~ /^stereo$|^mono$|orbslam/) orb += rss;
      if (comm_lower ~ /^gzserver$|^gzclient$|^gazebo$/) gazebo += rss;
      if (comm_lower ~ /^rviz2$/) rviz += rss;
      if (line ~ /pipeline_flow_bridge\.py|pipeline_flow_browser\.py/) web += rss;
    }
    ($1 + 0) != (pgid + 0) && ($2 + 0) > external_max_rss {
      external_max_rss = $2 + 0;
      external_max_comm = $4;
    }
    END {
      printf "%.0f,%.2f,%d,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%s,%.0f,%s\n",
        total_rss, total_cpu, count, server, orb, gazebo, rviz, web,
        max_rss, max_comm, external_max_rss, external_max_comm;
    }
  '
}

summarize_process_group_pss() {
  {
    while read -r pid process_pgid comm args; do
      if [ "$process_pgid" != "$PGID" ]; then
        continue
      fi
      read -r pss pss_anon pss_file pss_shmem <<< "$(awk '
        /^Pss:/ {pss=$2}
        /^Pss_Anon:/ {anon=$2}
        /^Pss_File:/ {file=$2}
        /^Pss_Shmem:/ {shmem=$2}
        END {print pss+0, anon+0, file+0, shmem+0}
      ' "/proc/$pid/smaps_rollup" 2>/dev/null)"
      printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
        "${pss:-0}" "${pss_anon:-0}" "${pss_file:-0}" "${pss_shmem:-0}" \
        "$comm" "$args"
    done < <(ps -eo pid=,pgid=,comm=,args= --no-headers 2>/dev/null)
  } | awk -F '\t' '
    BEGIN {
      total = server = orb = gazebo = rviz = web = 0;
      orb_anon = orb_file = orb_shmem = 0;
    }
    {
      pss = $1 + 0;
      pss_anon = $2 + 0;
      pss_file = $3 + 0;
      pss_shmem = $4 + 0;
      comm = tolower($5);
      line = tolower($0);
      total += pss;
      if (comm ~ /global_map_serv/ || line ~ /\/global_map_server([[:space:]]|$)/) server += pss;
      if (comm ~ /^stereo$|^mono$|orbslam/) {
        orb += pss;
        orb_anon += pss_anon;
        orb_file += pss_file;
        orb_shmem += pss_shmem;
      }
      if (comm ~ /^gzserver$|^gzclient$|^gazebo$/) gazebo += pss;
      if (comm ~ /^rviz2$/) rviz += pss;
      if (line ~ /pipeline_flow_bridge\.py|pipeline_flow_browser\.py/) web += pss;
    }
    END {
      printf "%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\n",
        total, server, orb, gazebo, rviz, web, orb_anon, orb_file, orb_shmem;
    }
  '
}

cat > "$CSV_FILE" <<'HEADER'
epoch_s,elapsed_s,mem_available_kib,mem_used_kib,swap_used_kib,memory_psi_some_avg10,memory_psi_full_avg10,cpu_psi_some_avg10,io_psi_full_avg10,cpu_busy_pct,cpu_iowait_pct,pswpin_delta_pages,pswpout_delta_pages,group_rss_kib,group_cpu_pct,group_processes,server_rss_kib,orb_rss_kib,gazebo_rss_kib,rviz_rss_kib,web_rss_kib,max_process_rss_kib,max_process_comm,max_external_process_rss_kib,max_external_process_comm,group_pss_kib,server_pss_kib,orb_pss_kib,gazebo_pss_kib,rviz_pss_kib,web_pss_kib,orb_pss_anon_kib,orb_pss_file_kib,orb_pss_shmem_kib
HEADER

start_epoch="$(date +%s)"
read -r prev_user prev_nice prev_system prev_idle prev_iowait prev_irq prev_softirq prev_steal \
  <<< "$(read_cpu_counters)"
prev_total=$((prev_user + prev_nice + prev_system + prev_idle + prev_iowait + prev_irq + prev_softirq + prev_steal))
prev_idle_all=$((prev_idle + prev_iowait))
prev_iowait_value="$prev_iowait"
prev_pswpin="$(read_vmstat_value pswpin)"
prev_pswpout="$(read_vmstat_value pswpout)"
guard_count=0
guard_reason="none"

while [ ! -f "$STOP_FILE" ]; do
  now_epoch="$(date +%s)"
  elapsed=$((now_epoch - start_epoch))
  mem_total="$(read_meminfo_kib MemTotal)"
  mem_available="$(read_meminfo_kib MemAvailable)"
  swap_total="$(read_meminfo_kib SwapTotal)"
  swap_free="$(read_meminfo_kib SwapFree)"
  mem_used=$((mem_total - mem_available))
  swap_used=$((swap_total - swap_free))

  memory_psi_some="$(read_pressure_avg10 /proc/pressure/memory some)"
  memory_psi_full="$(read_pressure_avg10 /proc/pressure/memory full)"
  cpu_psi_some="$(read_pressure_avg10 /proc/pressure/cpu some)"
  io_psi_full="$(read_pressure_avg10 /proc/pressure/io full)"
  memory_psi_some="${memory_psi_some:-0}"
  memory_psi_full="${memory_psi_full:-0}"
  cpu_psi_some="${cpu_psi_some:-0}"
  io_psi_full="${io_psi_full:-0}"

  read -r user nice system idle iowait irq softirq steal <<< "$(read_cpu_counters)"
  total=$((user + nice + system + idle + iowait + irq + softirq + steal))
  idle_all=$((idle + iowait))
  total_delta=$((total - prev_total))
  idle_delta=$((idle_all - prev_idle_all))
  iowait_delta=$((iowait - prev_iowait_value))
  if [ "$total_delta" -gt 0 ]; then
    cpu_busy_pct="$(awk -v total="$total_delta" -v idle="$idle_delta" 'BEGIN {printf "%.2f", 100 * (total - idle) / total}')"
    cpu_iowait_pct="$(awk -v total="$total_delta" -v wait="$iowait_delta" 'BEGIN {printf "%.2f", 100 * wait / total}')"
  else
    cpu_busy_pct="0.00"
    cpu_iowait_pct="0.00"
  fi
  prev_total="$total"
  prev_idle_all="$idle_all"
  prev_iowait_value="$iowait"

  pswpin="$(read_vmstat_value pswpin)"
  pswpout="$(read_vmstat_value pswpout)"
  pswpin_delta=$((pswpin - prev_pswpin))
  pswpout_delta=$((pswpout - prev_pswpout))
  prev_pswpin="$pswpin"
  prev_pswpout="$pswpout"

  process_values="$(summarize_process_group)"
  process_pss_values="$(summarize_process_group_pss)"
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "$now_epoch" "$elapsed" "$mem_available" "$mem_used" "$swap_used" \
    "$memory_psi_some" "$memory_psi_full" "$cpu_psi_some" "$io_psi_full" \
    "$cpu_busy_pct" "$cpu_iowait_pct" "$pswpin_delta" "$pswpout_delta" \
    "$process_values" "$process_pss_values" >> "$CSV_FILE"

  low_memory="$(awk -v kib="$mem_available" -v mib="$MIN_AVAILABLE_MIB" 'BEGIN {print (kib < mib * 1024 ? 1 : 0)}')"
  high_pressure="$(awk -v metric="$memory_psi_full" -v threshold="$MAX_MEMORY_PSI_FULL_AVG10" 'BEGIN {print (metric >= threshold ? 1 : 0)}')"
  if [ "$low_memory" -eq 1 ] || [ "$high_pressure" -eq 1 ]; then
    guard_count=$((guard_count + 1))
    if [ "$low_memory" -eq 1 ]; then
      guard_reason="mem_available_below_${MIN_AVAILABLE_MIB}_mib"
    else
      guard_reason="memory_psi_full_avg10_above_${MAX_MEMORY_PSI_FULL_AVG10}"
    fi
  else
    guard_count=0
    guard_reason="none"
  fi

  if [ "$guard_count" -ge "$GUARD_CONSECUTIVE" ]; then
    printf 'reason=%s elapsed_s=%s mem_available_kib=%s memory_psi_full_avg10=%s\n' \
      "$guard_reason" "$elapsed" "$mem_available" "$memory_psi_full" > "$GUARD_FILE"
    break
  fi

  sleep "$SAMPLE_SEC"
done

awk -F, -v guard_file="$GUARD_FILE" '
  NR == 2 {
    samples = 0;
    min_available = $3;
    max_mem_used = max_swap = max_mem_psi_some = max_mem_psi_full = 0;
    max_cpu_psi = max_io_psi = max_cpu = max_iowait = 0;
    max_pswpin = max_pswpout = max_group = max_group_cpu = 0;
    max_server = max_orb = max_gazebo = max_rviz = max_web = max_process = 0;
    max_external_process = 0;
    max_group_pss = max_server_pss = max_orb_pss = max_gazebo_pss = 0;
    max_rviz_pss = max_web_pss = 0;
    max_orb_pss_anon = max_orb_pss_file = max_orb_pss_shmem = 0;
  }
  NR > 1 {
    samples += 1;
    elapsed = $2;
    if ($3 < min_available) min_available = $3;
    if ($4 > max_mem_used) max_mem_used = $4;
    if ($5 > max_swap) max_swap = $5;
    if ($6 > max_mem_psi_some) max_mem_psi_some = $6;
    if ($7 > max_mem_psi_full) max_mem_psi_full = $7;
    if ($8 > max_cpu_psi) max_cpu_psi = $8;
    if ($9 > max_io_psi) max_io_psi = $9;
    if ($10 > max_cpu) max_cpu = $10;
    if ($11 > max_iowait) max_iowait = $11;
    if ($12 > max_pswpin) max_pswpin = $12;
    if ($13 > max_pswpout) max_pswpout = $13;
    if ($14 > max_group) max_group = $14;
    if ($15 > max_group_cpu) max_group_cpu = $15;
    if ($17 > max_server) max_server = $17;
    if ($18 > max_orb) max_orb = $18;
    if ($19 > max_gazebo) max_gazebo = $19;
    if ($20 > max_rviz) max_rviz = $20;
    if ($21 > max_web) max_web = $21;
    if ($22 > max_process) {max_process = $22; max_process_comm = $23;}
    if ($24 > max_external_process) {
      max_external_process = $24;
      max_external_process_comm = $25;
    }
    if ($26 > max_group_pss) max_group_pss = $26;
    if ($27 > max_server_pss) max_server_pss = $27;
    if ($28 > max_orb_pss) max_orb_pss = $28;
    if ($29 > max_gazebo_pss) max_gazebo_pss = $29;
    if ($30 > max_rviz_pss) max_rviz_pss = $30;
    if ($31 > max_web_pss) max_web_pss = $31;
    if ($32 > max_orb_pss_anon) max_orb_pss_anon = $32;
    if ($33 > max_orb_pss_file) max_orb_pss_file = $33;
    if ($34 > max_orb_pss_shmem) max_orb_pss_shmem = $34;
  }
  END {
    printf "samples=%d\n", samples;
    printf "duration_s=%d\n", elapsed;
    printf "min_mem_available_mib=%.1f\n", min_available / 1024;
    printf "max_mem_used_mib=%.1f\n", max_mem_used / 1024;
    printf "max_swap_used_mib=%.1f\n", max_swap / 1024;
    printf "max_memory_psi_some_avg10=%.2f\n", max_mem_psi_some;
    printf "max_memory_psi_full_avg10=%.2f\n", max_mem_psi_full;
    printf "max_cpu_psi_some_avg10=%.2f\n", max_cpu_psi;
    printf "max_io_psi_full_avg10=%.2f\n", max_io_psi;
    printf "max_system_cpu_pct=%.2f\n", max_cpu;
    printf "max_system_iowait_pct=%.2f\n", max_iowait;
    printf "max_pswpin_delta_pages=%.0f\n", max_pswpin;
    printf "max_pswpout_delta_pages=%.0f\n", max_pswpout;
    printf "max_group_rss_mib=%.1f\n", max_group / 1024;
    printf "max_group_cpu_pct=%.2f\n", max_group_cpu;
    printf "max_server_rss_mib=%.1f\n", max_server / 1024;
    printf "max_orb_rss_mib=%.1f\n", max_orb / 1024;
    printf "max_gazebo_rss_mib=%.1f\n", max_gazebo / 1024;
    printf "max_rviz_rss_mib=%.1f\n", max_rviz / 1024;
    printf "max_web_rss_mib=%.1f\n", max_web / 1024;
    printf "max_process_rss_mib=%.1f\n", max_process / 1024;
    printf "max_process_comm=%s\n", max_process_comm;
    printf "max_external_process_rss_mib=%.1f\n", max_external_process / 1024;
    printf "max_external_process_comm=%s\n", max_external_process_comm;
    printf "max_group_pss_mib=%.1f\n", max_group_pss / 1024;
    printf "max_server_pss_mib=%.1f\n", max_server_pss / 1024;
    printf "max_orb_pss_mib=%.1f\n", max_orb_pss / 1024;
    printf "max_gazebo_pss_mib=%.1f\n", max_gazebo_pss / 1024;
    printf "max_rviz_pss_mib=%.1f\n", max_rviz_pss / 1024;
    printf "max_web_pss_mib=%.1f\n", max_web_pss / 1024;
    printf "max_orb_pss_anon_mib=%.1f\n", max_orb_pss_anon / 1024;
    printf "max_orb_pss_file_mib=%.1f\n", max_orb_pss_file / 1024;
    printf "max_orb_pss_shmem_mib=%.1f\n", max_orb_pss_shmem / 1024;
    guard = (system("test -f \"" guard_file "\"") == 0) ? "true" : "false";
    printf "guard_triggered=%s\n", guard;
  }
' "$CSV_FILE" > "$SUMMARY_FILE"

if [ -f "$GUARD_FILE" ]; then
  cat "$GUARD_FILE" >> "$SUMMARY_FILE"
fi
