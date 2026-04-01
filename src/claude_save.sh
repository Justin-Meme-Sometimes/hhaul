#!/bin/bash
LOG_DIR="$HOME/.claude_logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/session_$(date +%Y%m%d_%H%M%S).log"
echo "Session log: $LOG_FILE"
script -q -a "$LOG_FILE" -c "claude"