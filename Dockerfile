FROM golang:1.24-alpine AS builder

WORKDIR /src

RUN apk add --no-cache git

COPY go.mod go.sum* ./
RUN /usr/local/go/bin/go mod download

COPY . .
RUN CGO_ENABLED=0 GOOS=linux GOARCH=amd64 /usr/local/go/bin/go build -trimpath -ldflags="-s -w" -o /out/deif-ha-bridge .

FROM alpine:3.23

WORKDIR /app

ENV SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt

RUN apk add --no-cache ca-certificates \
  && addgroup -g 20 -S dialout || true \
  && adduser -S -D -H -u 65532 nonroot \
  && addgroup nonroot dialout

COPY --from=builder /out/deif-ha-bridge /app/deif-ha-bridge

USER nonroot

STOPSIGNAL SIGTERM

ENTRYPOINT ["/app/deif-ha-bridge"]
