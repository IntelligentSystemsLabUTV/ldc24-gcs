The server rest works in the following way:

- add an example json file in the same path as the script. Example: milano_manche1.json
- run the server, adding ip, port and file name. 
Example: python3 server_rest.py --ip 192.168.142.218 --port 8080 --file_name milano_manche1

From another terminal run: wget http://192.168.142.218:8080 -O download_mission_manche1.json
