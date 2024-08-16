import ephem
import telepot
from telepot.loop import MessageLoop
from time import sleep
from datetime import datetime

latitude = None
longitude = None

# Replace with your actual chat_id
user_chat_id = '7048803908'

def handle(msg, bot):
    global latitude, longitude
    content_type, chat_type, chat_id = telepot.glance(msg)

    if content_type == 'location':
        latitude = msg['location']['latitude']
        longitude = msg['location']['longitude']
        print(f'Latitude: {latitude}, Longitude: {longitude}')
        bot.sendMessage(chat_id, f'Your location is: Latitude {latitude}, Longitude {longitude}')
        get_sun_position(latitude, longitude)
    else:
        bot.sendMessage(chat_id, 'Please share your location.')

def get_sun_position(lat, lon):
    obs = ephem.Observer()
    obs.lat = str(lat)
    obs.lon = str(lon)
    obs.elevation = 0

    current_time = datetime.now()
    formatted_time = current_time.strftime('%Y/%m/%d %H:%M:%S')
    obs.date = formatted_time

    sun = ephem.Sun(obs)
    sun.compute(obs)
    azimuth = sun.az
    altitude = sun.alt

    # Write azimuth and altitude to the file
    with open('sun_position.txt', 'w') as file:
        file.write(f"{azimuth}\n{altitude}\n")

    # Create the flag file
    with open('update_flag.txt', 'w') as flag_file:
        flag_file.write('Sun position updated')

    print("Azimuth:", azimuth)
    print("Altitude:", altitude)

def start_bot():
    bot = telepot.Bot('7385084143:AAGaZyrcoGdHQQ8P_oVFawk4bE4myKlhhuM')
    MessageLoop(bot, lambda msg: handle(msg, bot)).run_as_thread()

    # Wait a bit to ensure the bot is ready to send messages
    sleep(5)

    # Send a message requesting location to the fixed chat_id
    bot.sendMessage(user_chat_id, 'Please share your location so I can provide you with the sun position.')

    while True:
        sleep(10)  # Replace with any other necessary activity

if __name__ == '__main__':
    start_bot()

try:
    while True:
        pass
        # Uncomment and modify these lines if needed for additional functionality
        # a, b = get_sun_position()
        # string = f"{a},{b}\r"
        # ser.write(string.encode())
        # sleep(1)
except KeyboardInterrupt:
    # Ensure to close any resources if needed (e.g., serial connection)
    # ser.close()
    pass