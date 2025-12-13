# MFE-Driverless-V1

Welcome to the McGill Formula Electric Driverless Software repository! In this repository you can find all the required packages to boot and operate an NVIDIA Jetson with the MFE2X car. 

This page is still under construction. Stay tuned!


Save the API key using the following command:
```bash
export DOCKERKEY_PERM=tskey-auth-1234567890abcdef
```

Run this command inside the Docker folder to run and start the docker image: 

```bash
docker compose -f dev/docker-compose.yml up --build -d
```
