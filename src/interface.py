import matplotlib.pyplot as plt
import seaborn as sns
import asyncio
import threading
from collections import deque
from bleak import BleakClient
from matplotlib.widgets import Button

# Config BLE
ADDRESS = "F8:B3:B7:33:2F:62"
CHARACTERISTIC_UUID = "7a6ffc80-ef27-4a4d-a8a6-56d93f8feff3"

# Dades
data_queue = deque()
data_lock = threading.Lock()
dataC = deque([0.0]*150, maxlen=150)
dataR = deque([0.0]*150, maxlen=150)
dataSN = [0.0, 0.0]
nivell_estres = 0.0

# Flag per parar
stop_flag = False

# Estètica
sns.set_theme(style="darkgrid")
fig, axs = plt.subplots(2, 2, figsize=(10, 6))
fig.subplots_adjust(wspace=0.4, bottom=0.15)
fig.subplots_adjust(hspace=0.6)
fig.suptitle('Monitorització Fisiològica', fontsize=18, fontweight='bold')

lineC, = axs[0, 0].plot(dataC, color='tab:red', linewidth=1.5)
axs[0, 0].set_ylim(-5, 5)
axs[0, 0].set_title("Senyal cardíaca")

lineR, = axs[1, 0].plot(dataR, color='tab:blue', linewidth=1.5)
axs[1, 0].set_ylim(0, 100)
axs[1, 0].set_title("Senyal respiratòria")

barSN = axs[0, 1].bar(['SNS', 'SNP'], dataSN, color=['tab:green', 'tab:purple'], width=0.4)
axs[0, 1].set_ylim(0, 100)
axs[0, 1].set_title("Sistema nerviós")

axs[1, 1].axis('off')
text_display = axs[1, 1].text(0.5, 0.5, "0%", fontsize=40, ha='center', va='center', color='darkred')

# Boto aturada
stop_ax = plt.axes([0.8, 0.05, 0.15, 0.075])
stop_button = Button(stop_ax, 'Aturar', color='red', hovercolor='salmon')

def stop_program(event):
    global stop_flag
    print("Aturant el programa...")
    stop_flag = True
    plt.close()

stop_button.on_clicked(stop_program)

# Callback BLE
def callback(sender, data):
    try:
        decoded = data.decode("utf-8").strip()
        if decoded:
            print("[BLE REBUT]>", decoded)
            with data_lock:
                data_queue.append(decoded)
    except Exception as e:
        print("[ERROR DE CODIFICACIÓ BLE]:", e)

# BLE async task
async def run_ble():
    async with BleakClient(ADDRESS) as client:
        print("Connectat al dispositiu...")
        await client.start_notify(CHARACTERISTIC_UUID, callback)
        await asyncio.sleep(0.1)
        while not stop_flag:
            await asyncio.sleep(0.1)

def start_ble_thread():
    asyncio.run(run_ble())

# Iniciem fil BLE
ble_thread = threading.Thread(target=start_ble_thread, daemon=True)
ble_thread.start()

# MODE INTERACTIU MATPLOTLIB
plt.ion()
plt.show()

# Bucle principal de gràfic al MAIN THREAD
try:
    while not stop_flag:
        noves_dades = []
        with data_lock:
            while data_queue:
                noves_dades.append(data_queue.popleft())

        for paquet in noves_dades:
            parts = paquet.split(",")

            if len(parts) == 5:
                try:
                    valC, valR, valSNS, valSNP, estres = map(float, parts)
                    dataC.append(valC)
                    dataR.append(valR)
                    dataSN[0], dataSN[1] = valSNS, valSNP
                    nivell_estres = estres
                except ValueError as e:
                    print("[ERROR DE CONVERSIÓ A FLOAT]:", parts, "|", e)
                    continue
            else:
                print("[FORMAT INVÀLID] Paquet rebut amb", len(parts), "elements:", parts)

        # Actualització gràfica
        lineC.set_ydata(dataC)
        lineR.set_ydata(dataR)

        for rect, val in zip(barSN.patches, dataSN):
            rect.set_height(val)

        color = 'green' if nivell_estres < 50 else 'orange' if nivell_estres < 75 else 'red'
        text_display.set_color(color)
        text_display.set_text(f"{nivell_estres:.1f}%")

        plt.pause(0.05)

except KeyboardInterrupt:
    print("Programa aturat per teclat.")
    stop_flag = True
finally:
    ble_thread.join()
    print("Programa finalitzat.")
