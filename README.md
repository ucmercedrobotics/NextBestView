To start, make your local Docker network to connect your VNC client, local machine, and Amiga together. You'll use this later when remote controlling the Amiga.
```bash
make network
```

After, build your container:
```bash
make build-image
```

Next, standup the VNC container to forward X11 to your web browser. You can see this at `localhost:8080`.
```bash
make vnc
```

Finally, standup your container:
```bash
make bash
```