import requests

for i in range(100):
    r = requests.get("http://192.168.4.1/capture")
    open(f"{i}.jpg", "wb").write(r.content)