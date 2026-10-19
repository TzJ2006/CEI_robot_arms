"""Run the policy link from team_config.json.

Operators use three commands in this folder:

    python team_config.py run
    python team_config.py once
    python team_config.py down

`run` submits or reuses the scheduler job, waits until the server log says it
is listening, opens one local SSH forward to that compute node, then sends
one observation. `once` sends another observation while that forward is up.
`down` cancels the serve-policy job and closes the local forward.

Account, code directory, and checkpoint stay in team_config.json.
"""

import base64
import json
import shlex
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CONFIG_PATH = ROOT / "team_config.json"
STATE_PATH = ROOT / "piper_link_state.json"
PLACEHOLDER_PORT = 8765
WAIT_POLLS = 180
WAIT_SECONDS = 10


def load():
    with CONFIG_PATH.open(encoding="utf-8") as handle:
        return json.load(handle)


def require(config, keys):
    missing = [key for key in keys if config.get(key) in ("", None)]
    if missing:
        joined = ", ".join(missing)
        raise SystemExit(f"Fill these keys in team_config.json: {joined}")


def configured_port(config=None):
    if config is None:
        try:
            config = load()
        except (OSError, json.JSONDecodeError):
            return PLACEHOLDER_PORT
    value = config.get("local_forward_port")
    if value in ("", None):
        return PLACEHOLDER_PORT
    return int(value)


def serve_command(config):
    require(config, ["openpi_dir", "policy_config", "checkpoint_dir", "local_forward_port"])
    return f"""cd {config["openpi_dir"]}
.venv/bin/python scripts/serve_policy.py --port {config["local_forward_port"]} policy:checkpoint \\
  --policy.config={config["policy_config"]} \\
  --policy.dir={config["checkpoint_dir"]}"""


def job_script(config):
    require(
        config,
        ["log_dir", "slurm_partition", "slurm_time", "slurm_mem", "remote_user"],
    )
    cd_line, command = serve_command(config).split("\n", 1)
    return f"""mkdir -p {config["log_dir"]}
sbatch <<'EOF'
#!/bin/bash
#SBATCH --job-name=serve-policy
#SBATCH --partition={config["slurm_partition"]}
#SBATCH --gres=gpu:{config["slurm_gpus"]}
#SBATCH --cpus-per-task={config["slurm_cpus"]}
#SBATCH --mem={config["slurm_mem"]}
#SBATCH --time={config["slurm_time"]}
#SBATCH --output={config["log_dir"]}/serve-policy-%j.out
{cd_line}
exec {command}
EOF"""


def ssh_argv(config):
    if config.get("ssh_host"):
        return ["ssh", config["ssh_host"]]
    require(config, ["login_host", "remote_user"])
    return [
        "ssh",
        "-o",
        "StrictHostKeyChecking=accept-new",
        "-l",
        config["remote_user"],
        config["login_host"],
    ]


def remote_up_script(config):
    require(
        config,
        [
            "remote_user",
            "log_dir",
            "openpi_dir",
            "policy_config",
            "checkpoint_dir",
            "local_forward_port",
            "slurm_partition",
            "slurm_time",
            "slurm_mem",
        ],
    )
    port = int(config["local_forward_port"])
    return f"""set -eu
LOG_DIR={shlex.quote(str(config["log_dir"]))}
PORT={port}
ACCOUNT={shlex.quote(str(config["remote_user"]))}
OPENPI={shlex.quote(str(config["openpi_dir"]))}
POLICY={shlex.quote(str(config["policy_config"]))}
CKPT={shlex.quote(str(config["checkpoint_dir"]))}
PARTITION={shlex.quote(str(config["slurm_partition"]))}
SLURM_TIME={shlex.quote(str(config["slurm_time"]))}
MEM={shlex.quote(str(config["slurm_mem"]))}
GPUS={int(config["slurm_gpus"])}
CPUS={int(config["slurm_cpus"])}
mkdir -p "$LOG_DIR"
existing=$(squeue -u "$ACCOUNT" -h -n serve-policy -o "%i %T %N" | sed -n '1p' || true)
if [ -n "$existing" ]; then
  JOB_ID=${{existing%% *}}
  echo "submitted_job=$JOB_ID reused=1"
else
  JOB_ID=$(sbatch --parsable <<EOF
#!/bin/bash
#SBATCH --job-name=serve-policy
#SBATCH --partition=$PARTITION
#SBATCH --gres=gpu:$GPUS
#SBATCH --cpus-per-task=$CPUS
#SBATCH --mem=$MEM
#SBATCH --time=$SLURM_TIME
#SBATCH --output=$LOG_DIR/serve-policy-%j.out
cd "$OPENPI"
exec .venv/bin/python scripts/serve_policy.py --port "$PORT" policy:checkpoint --policy.config="$POLICY" --policy.dir="$CKPT"
EOF
)
  JOB_ID=${{JOB_ID%%;*}}
  echo "submitted_job=$JOB_ID reused=0"
fi
for _ in $(seq 1 {WAIT_POLLS}); do
  line=$(squeue -h -j "$JOB_ID" -o "%T %N" || true)
  if [ -z "$line" ]; then
    echo "job_ended job=$JOB_ID"
    tail -n 40 "$LOG_DIR/serve-policy-$JOB_ID.out" || true
    exit 1
  fi
  state=${{line%% *}}
  node=${{line#* }}
  log="$LOG_DIR/serve-policy-$JOB_ID.out"
  if [ "$state" = "RUNNING" ] && [ -f "$log" ] && grep -q "listening on" "$log"; then
    addr=$(scontrol show node "$node" | sed -n 's/.*NodeAddr=\\([^ ]*\\).*/\\1/p' | sed -n '1p' || true)
    if [ -z "$addr" ]; then
      addr=$node
    fi
    echo "ready job=$JOB_ID node=$node addr=$addr"
    exit 0
  fi
  echo "waiting state=$state" >&2
  sleep {WAIT_SECONDS}
done
echo "timeout job=$JOB_ID"
tail -n 40 "$LOG_DIR/serve-policy-$JOB_ID.out" || true
exit 1
"""


def parse_ready(output):
    ready = None
    for line in output.splitlines():
        if line.startswith("ready "):
            fields = {}
            for item in line.split()[1:]:
                key, _, value = item.partition("=")
                fields[key] = value
            ready = fields
    return ready


def run_ssh(config, remote):
    print("Connecting to the login node. Type the password in this window; nothing is shown as you type.", flush=True)
    payload = base64.b64encode(remote.encode()).decode()
    command = f"echo {payload} | base64 -d | stdbuf -oL -eL bash"
    process = subprocess.Popen(
        ssh_argv(config) + [command],
        stdout=subprocess.PIPE,
        stderr=None,
        text=True,
    )
    captured = []
    assert process.stdout is not None
    for line in process.stdout:
        print(line, end="", flush=True)
        captured.append(line)
    code = process.wait()
    output = "".join(captured)
    if code != 0:
        raise SystemExit(code)
    return output


def local_port_open(port):
    try:
        with socket.create_connection(("127.0.0.1", int(port)), timeout=1):
            return True
    except OSError:
        return False


def wait_for_local_port(port):
    deadline = time.time() + 180
    while time.time() < deadline:
        if local_port_open(port):
            return True
        time.sleep(1)
    return False


def cmd_quote(arg):
    if arg == "" or any(char in arg for char in ' \t&|<>^"'):
        return '"' + arg.replace('"', '""') + '"'
    return arg


def shell_join(args):
    return " ".join(cmd_quote(arg) for arg in args)


def open_tunnel(config, addr):
    port = configured_port(config)
    if local_port_open(port):
        print(f"Local port {port} is already forwarded; reusing it.", flush=True)
        return
    forward = f"{port}:{addr}:{port}"
    command = shell_join(ssh_argv(config) + ["-N", "-L", forward])
    print(
        "A window titled piper-link will open. Type the SSH password there once more, then leave that window open.",
        flush=True,
    )
    subprocess.Popen(f'start "piper-link" cmd /k {command}', shell=True, cwd=str(ROOT))
    if not wait_for_local_port(port):
        raise SystemExit(f"Local port {port} did not accept a connection within three minutes. Type the password in the piper-link window, then run python team_config.py once")
    print(f"Local port {port} is forwarded to {addr}.", flush=True)


def save_state(ready, port):
    STATE_PATH.write_text(
        json.dumps(
            {
                "job_id": ready.get("job", ""),
                "node": ready.get("node", ""),
                "addr": ready.get("addr", ""),
                "port": port,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )


def up(config):
    output = run_ssh(config, remote_up_script(config))
    ready = parse_ready(output)
    if ready is None or not ready.get("addr"):
        raise SystemExit("The login node did not return a compute-node address. The log above shows why the job exited or timed out.")
    save_state(ready, configured_port(config))
    open_tunnel(config, ready["addr"])


def once(config):
    port = configured_port(config)
    if not local_port_open(port):
        raise SystemExit(f"Local port {port} is not forwarded. Run python team_config.py up first")
    raise SystemExit(
        subprocess.call(
            [sys.executable, str(ROOT / "policy_link.py"), "once", "--host", "127.0.0.1", "--port", str(port)]
        )
    )


def read_arm():
    from piper_client import PiperClient, format_state

    robot = PiperClient()
    robot.connect()
    try:
        print(format_state(robot.read_state()))
    finally:
        robot.close()


def stop_local_tunnels(port):
    script = (
        "Get-CimInstance Win32_Process -Filter \"Name='ssh.exe'\" | "
        f"Where-Object {{ $_.CommandLine -like '*-N*' -and $_.CommandLine -like '*-L {port}:*' }} | "
        "ForEach-Object { Stop-Process -Id $_.ProcessId -Force }"
    )
    subprocess.run(["powershell", "-NoProfile", "-Command", script], check=False)


def down(config):
    require(config, ["remote_user"])
    account = shlex.quote(str(config["remote_user"]))
    remote = f"""set -eu
ids=$(squeue -u {account} -h -n serve-policy -o %i || true)
if [ -z "$ids" ]; then
  echo "no_job"
  exit 0
fi
scancel $ids
echo "cancelled $ids"
"""
    output = run_ssh(config, remote)
    stop_local_tunnels(configured_port(config))
    if STATE_PATH.exists():
        STATE_PATH.unlink()
    if "no_job" in output:
        print("No serve-policy job is running. The local forward is closed.", flush=True)
    else:
        print("Cancelled the serve-policy job and closed the local forward.", flush=True)


def run(config):
    up(config)
    once(config)


def show(config):
    for key, value in config.items():
        shown = value if value not in ("", None) else "empty"
        print(f"{key}={shown}")


def main():
    parser = argparse_parser()
    args = parser.parse_args()
    config = load()
    if args.mode == "run":
        run(config)
    elif args.mode == "up":
        up(config)
    elif args.mode == "once":
        once(config)
    elif args.mode == "down":
        down(config)
    elif args.mode == "arm":
        read_arm()
    elif args.mode == "job":
        print(job_script(config))
    elif args.mode == "serve":
        print(serve_command(config))
    elif args.mode == "show":
        show(config)
    else:
        raise SystemExit(f"unknown mode {args.mode}")


def argparse_parser():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("run", "up", "once", "down", "arm", "job", "serve", "show"))
    return parser


if __name__ == "__main__":
    main()
