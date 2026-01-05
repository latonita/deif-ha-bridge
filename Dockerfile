FROM node:20-bookworm-slim AS builder

# Native build deps for serialport/modbus-serial
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 make g++ \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install deps first for better caching
COPY package.json package-lock.json* ./
RUN npm ci --omit=dev && npm cache clean --force

# Copy app (no need for other files in final image)
COPY deif_to_mqtt.js ./

# Prepare minimal passwd/group with dialout for serial devices
RUN set -eux; \
  cp /etc/passwd /tmp/passwd; \
  cp /etc/group /tmp/group; \
  echo 'nonroot:x:65532:65532:nonroot:/home/nonroot:/sbin/nologin' >> /tmp/passwd; \
  echo 'nonroot:x:65532:' >> /tmp/group; \
  echo 'dialout:x:20:nonroot' >> /tmp/group

FROM gcr.io/distroless/nodejs20-debian12:nonroot
ENV NODE_ENV=production
WORKDIR /app

# Copy runtime artifacts only
COPY --from=builder /app/node_modules ./node_modules
COPY --from=builder /app/package.json ./package.json
COPY --from=builder /app/deif_to_mqtt.js ./deif_to_mqtt.js
COPY --from=builder /tmp/passwd /etc/passwd
COPY --from=builder /tmp/group /etc/group

CMD ["deif_to_mqtt.js"]
