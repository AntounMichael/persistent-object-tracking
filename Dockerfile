# Use Ubuntu 22.04 as base image
FROM ubuntu:22.04

# Set work directory
WORKDIR /app

ENV DEBIAN_FRONTEND=noninteractive
# Install dependencies (add OpenCV)
RUN apt-get update && \
    apt-get install -y g++ cmake libeigen3-dev libopencv-dev git && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Copy source code
COPY . .

# Clone the munkres-cpp library into /app/external/munkres-cpp
RUN git clone https://github.com/saebyn/munkres-cpp.git external/munkres-cpp

# Build the tracker and test data generator in /app/build
RUN mkdir -p build \
    && cd build \
    && cmake .. \
    && make

# Generate test data in /app/data
RUN mkdir -p /app/data && /app/build/generate_test_data

# Set entrypoint to the tracking solution in /app/build
ENTRYPOINT ["/app/build/tracking-solution"]
