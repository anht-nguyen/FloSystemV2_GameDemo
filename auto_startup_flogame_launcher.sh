#!/bin/bash
cd /home/$USER/Documents/git/FloSystemV2_GameDemo  # adjust if different

# Make sure docker group is set correctly
newgrp docker <<EONG
  docker compose -f docker-compose.yml up --build -d
EONG
