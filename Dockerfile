FROM node:20-bookworm-slim

# Native build deps for serialport/modbus-serial
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 make g++ \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install deps first for better caching
COPY package.json package-lock.json* ./
RUN npm ci --omit=dev || npm install --omit=dev

# Copy app
COPY deif_to_mqtt.js ./

# Non-root user (recommended)
RUN useradd -m -u 10001 appuser
USER appuser

CMD ["node", "deif_to_mqtt.js"]
