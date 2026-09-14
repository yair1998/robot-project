FROM ubuntu:24.04

# Everything needed to compile the C++ agent and generate gRPC code from
# the shared .proto file - all pulled straight from Ubuntu's own repos,
# no vcpkg, no building grpc from source.
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    protobuf-compiler \
    protobuf-compiler-grpc \
    libprotobuf-dev \
    libgrpc++-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Keeps the container running so you can `docker compose exec` into it
# and build/run things interactively while you're writing code.
CMD ["sleep", "infinity"]
