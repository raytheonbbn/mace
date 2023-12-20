#!/bin/bash

openssl req -x509 -days 10000 -nodes -newkey rsa:4096 -keyout ca_certificates/ca.key -out ca_certificates/ca.crt --config openssl.cnf
openssl req -nodes -newkey rsa:4096 -keyout certs/server.key -out certs/server.csr --config openssl.cnf
openssl x509 -req -in certs/server.csr -days 10000 -CA ca_certificates/ca.crt -CAkey ca_certificates/ca.key -out certs/server.crt

