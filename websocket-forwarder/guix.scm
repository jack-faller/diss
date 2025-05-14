(use-modules (guix gexp)
			 (guix packages)
			 (guix git-download)
			 (guix build-system gnu)
			 ((guix licenses) #:prefix license:)

			 (gnu packages tls)
			 (gnu packages pkg-config)
			 (gnu packages web))

(define websocket-forwarder
  (package
    (name "websocket-forwarder")
    (version "0.0.0")
    (source (local-file "." "src" #:recursive? #t))
    (build-system gnu-build-system)
    (arguments
     `(#:phases (modify-phases %standard-phases
                  (delete 'configure)
				  (replace 'install
					(lambda* (#:key outputs #:allow-other-keys)
					  (let* ((out (assoc-ref outputs "out"))
							 (bin (string-append out "/bin")))
						(install-file "websocket-forwarder" bin)))))
       #:tests? #f))
    (inputs (list libwebsockets openssl pkg-config))
    (home-page "")
    (synopsis "")
    (description "")
    (license license:expat)))
websocket-forwarder
