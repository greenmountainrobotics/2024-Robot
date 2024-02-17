#!/usr/bin/env sh

# This script should be run automatically every time robot code is built at competition.
# Create a new branch for competition (title = "event_" ++ official event acronym ++ "_" ++ year, when possible)
# EX: "event_MAWOR_2024"
# When competition is over, SQUASH and merge, but keep branch in GitHub for later replay.

case $(git branch --show-current) in event_*);;*)
echo "ERROR: not in a competition branch (\"event_COMPNAME_YYYY\")"
exit 1
esac

git add .
git commit -m "updates @ $(date -Iseconds)" || true