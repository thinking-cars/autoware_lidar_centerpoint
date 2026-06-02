# update CUDA Toolkit
apt update && apt install -y --no-install-recommends cuda-toolkit-12-8 || true
update-alternatives --install /usr/local/cuda cuda /usr/local/cuda-12.8 128
update-alternatives --set cuda /usr/local/cuda-12.8
rm -rf /var/lib/apt/lists/*
rm -r /usr/local/cuda-12.6

du --max-depth=5 / | sort -r -k1,1n
