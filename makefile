run:
	go run .

build-linux-to-all: build-linux-to-linux build-linux-to-windows

build-linux-to-linux:
	GOARCH=amd64 GOOS=linux go build -o bin/boids-linux

build-linux-to-windows:
	GOARCH=386 GOOS=windows CGO_ENABLED=1 CC=i686-w64-mingw32-gcc go build -o bin/boids-windows.exe