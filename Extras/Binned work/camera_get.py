from selenium import webdriver
from selenium.webdriver.common.by import By
import time
import urllib.request
import requests

driver = webdriver.Firefox()

driver.get("http://192.168.4.1/")
time.sleep(1)

driver.maximize_window()
time.sleep(5)

count = 0

for i in range(100):
    button = driver.find_element(By.ID,'get-still')
    button.click()

    time.sleep(2)

    img = driver.find_element(By.ID, 'stream')
    src = img.get_attribute('src')
    print(src)

    # with open(f"{count}.png", "wb") as file:
    #     file.write(requests.get(src).content)
    
    with open(f"{count}.png", 'wb') as file:
        file.write(driver.find_element(By.ID, 'stream').screenshot_as_png)
        #file.write(data)
    

    count += 1

driver.close()




