# Docker dev environment

## Build Container

> [!NOTE] 
> All commands sould be ran from the docker folder (this folder)

Before building remember to set ROS_DOMAIN_ID, it defaults to 0.

```sh
# change [ID] to your desired ROS_DOMAIN_ID
export ROS_DOMAIN_ID=[ID]
```

### Method 1

Use premade script to build container, run the build script.

```sh
sudo chmod +x ./build.sh
./build.sh
```

### Method 2

Add required patameters to your bashrc file and use docker compose command.

- Add required arguments to .bashrc (ONLY FIRST TIME)
  ```sh 
  echo "export USER_ID=$(id -u)" >> ~/.bashrc
  echo "export GROUP_ID=$(id -g)" >> ~/.bashrc
  ```

- Build container
  ```sh
  docker compose build
  ```

## Run and Connect to Container

Run container with compose
```sh
docker compose up -d
```
Then connect to container
```sh
docker exec -it slarc_dev bash
```

>[!NOTE]
>If you want to show any graphical interfaces from the container you need to allow xhost. Simpliest is to allow all connections to xhost. Open a new terminal outside the docker container and run:
>  ```sh
>  # Allow all xhost connections
>  xhost +
>  ```


## shut down container
```sh
docker compose down
```
