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
		if (req.method === 'POST' && req.url.startsWith('/dissert/')) {
			let data = '';
			req.on('data', chunk => {
				data += chunk.toString();
			});
			req.on('end', () => {
				res.end('Data received');
				sock.write(req.url.substring('/dissert/'.length)
						   + ' ' + data + '\r\n')
			});
		} else if (req.method == 'GET'
				   && (req.url == '/dissert' || req.url == '/dissert/')) {
			fs.readFile(process.argv[2], function(error, content) {
				res.writeHead(200, { 'Content-Type': 'text/html' });
				res.end(content, 'utf-8');
			});
		} else {
			res.end('Bad request '
					+ req.method + ' ' + req.url);
		}
	});

	sock.on('end', () => {
		server.close();
		listening = false;
		console.log('Server closed');
	})

	server.listen(3001, () => {
		console.log('Server running on port 3001');
	});
});

tcp_server.listen(3000)
