from selenium import webdriver
from selenium.webdriver.common.by import By
import time

driver = webdriver.Firefox()

driver.get("http://192.168.4.1/capture")
img = driver.find_element(By.TAG_NAME, 'img')
driver.maximize_window()
time.sleep(5)

for i in range(200):
    img = driver.find_element(By.TAG_NAME, 'img')
    img.screenshot(f"pics/{i}.png")
    time.sleep(1)

    driver.refresh()

driver.close()




# import requests, time




# for i in range(100):
#     r = requests.get("http://192.168.4.1/capture", timeout=5)
#     open(f"{i}.jpg", "wb").write(r.content)
#     time.sleep(2)