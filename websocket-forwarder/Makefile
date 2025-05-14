all:
	gcc main.c -o websocket-forwarder $$(pkg-config --cflags --libs libwebsockets)
websocket-forwarder.AppImage: main.c
	ln -fs $$(guix pack -f appimage --entry-point=bin/websocket-forwarder --relocatable --file=guix.scm) \
		$@
send: websocket-forwarder.AppImage
	scp $< root@jackfaller.xyz:/bin
.PHONY: all pack send
