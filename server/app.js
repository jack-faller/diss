const http = require('http');
const net = require('net');
const fs = require('fs');

listening = false;

const tcp_server = net.createServer(sock => {
	if (listening) {
		console.log('Duplicate listen attempt');
		sock.close();
	}
	// 'connect' listener.
	console.log('Connected');
	listening = true;

	const server = http.createServer((req, res) => {
		if (req.method === 'POST') {
			let data = '';
			req.on('data', chunk => {
				data += chunk.toString();
			});
			req.on('end', () => {
				console.log('POST data:', data);
				res.end('Data received');
				sock.write(req.url + ' ' + data + '\r\n')
			});
		} else if (req.method == 'GET') {
			fs.readFile("control.html", function(error, content) {
				res.writeHead(200, { 'Content-Type': 'text/html' });
				res.end(content, 'utf-8');
			});
		} else {
			res.end('Unknown request method');
		}
	});

	sock.on('end', () => {
		server.close();
		listening = false;
	})

	server.listen(3001, () => {
		console.log('Server running on port 3001');
	});
});

tcp_server.listen(3000)
