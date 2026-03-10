import matplotlib.pyplot as plt
import seaborn as sns
import asyncio
import threading
from collections import deque
from bleak import BleakClient
from matplotlib.widgets import Button
import time

# Configuració BLE
ADDRESS = "F8:B3:B7:33:2F:62"
CHARACTERISTIC_UUID = "7a6ffc80-ef27-4a4d-a8a6-56d93f8feff3"

# Variables de dades
dataQueue = deque()
dataLock = threading.Lock()
maxPoints = 4500  # Nombre màxim de punts a mostrar
dataCardiaca = deque(maxlen=maxPoints)  # Dades cardíaques
dataRespiratoria = deque(maxlen=maxPoints)  # Dades respiratòries
timeStamps = deque(maxlen=maxPoints)  # Marques de temps
dataSistemaNervios = [0.0, 0.0]  # Dades SNS i SNP
nivellEstres = 0.0  # Nivell d'estrès calculat

# Control d'execució
stopFlag = False  # Flag per aturar l'execució

# Configuració temporal
startTime = time.time()
windowDurationCardiaca = 30  # Finestra temporal per senyal cardíaca (segons)
windowDurationRespiratoria = 60  # Finestra temporal per senyal respiratòria (segons)

## Configuració de la interfície gràfica
sns.set_theme(style="darkgrid")
fig, axs = plt.subplots(2, 2, figsize=(10, 6))
fig.suptitle('Monitorització Fisiològica', fontsize=18, fontweight='bold')
plt.subplots_adjust(hspace=0.4, wspace=0.3)

# Gràfic senyal cardíaca
lineCardiaca, = axs[0, 0].plot([], [], color='tab:red', linewidth=1.5)
axs[0, 0].set_xlim(0, windowDurationCardiaca)
axs[0, 0].set_ylim(-1.5, 1.5)
axs[0, 0].set_title("Senyal cardíaca", fontsize=14, fontweight='bold')
axs[0, 0].set_xlabel("Temps (s)")

# Gràfic senyal respiratòria
lineRespiratoria, = axs[1, 0].plot([], [], color='tab:blue', linewidth=1.5)
axs[1, 0].set_xlim(0, windowDurationRespiratoria)
axs[1, 0].set_ylim(0, 10)
axs[1, 0].set_title("Senyal respiratòria", fontsize=14, fontweight='bold')
axs[1, 0].set_xlabel("Temps (s)")

# Gràfic sistema nerviós
barSistemaNervios = axs[0, 1].bar(['SNS', 'SNP'], dataSistemaNervios, 
                                 color=['tab:green', 'tab:purple'], width=0.4)
axs[0, 1].set_ylim(0, 100)
axs[0, 1].set_title("Activitat sistema nerviós", fontsize=14, fontweight='bold')

# Display nivell d'estrès
axs[1, 1].axis('off')
axs[0, 1].set_title("Nivell d'estrès", fontsize=14, fontweight='bold')
textDisplay = axs[1, 1].text(0.5, 0.5, "0%", fontsize=40, 
                            ha='center', va='center', color='darkred')

# Botó d'aturada
stopAx = plt.axes([0.8, 0.01, 0.1, 0.05])
stopButton = Button(stopAx, 'Aturar', color='red', hovercolor='salmon')

def aturar(event=None):
    """Funció que s'executa quan es prem el botó d'aturada o es tanca la finestra"""
    global stopFlag
    print("S'ha premut el botó d'aturada o s'ha tancat la finestra.")
    stopFlag = True
    plt.close()

stopButton.on_clicked(aturar)

def callback(sender, data):
    """Funció callback que rep dades del dispositiu BLE
    
    Args:
        sender: Identificador del dispositiu BLE
        data: Dades rebudes en format bytes
    """
    try:
        decoded = data.decode("utf-8").strip()
        if decoded:
            with dataLock:
                dataQueue.append(decoded)
    except Exception as e:
        print("[ERROR DE CODIFICACIÓ BLE]:", e)

async def runBle():
    """Funció asíncrona per gestionar la connexió BLE"""
    async with BleakClient(ADDRESS) as client:
        print("Connectat al dispositiu...")
        await client.start_notify(CHARACTERISTIC_UUID, callback)
        while not stopFlag:
            await asyncio.sleep(0.1)

def startBleThread():
    """Inicia el fil d'execució per a la connexió BLE"""
    asyncio.run(runBle())

# Iniciem el fil BLE
bleThread = threading.Thread(target=startBleThread, daemon=True)
bleThread.start()

# Configuració per detectar tancament de finestra
fig.canvas.mpl_connect('close_event', aturar)

# Activem mode interactiu
#plt.ion()

## Bucle principal de visualització
try:
    while not stopFlag:
        # Obtenim noves dades de la cua
        novesDades = []
        with dataLock:
            while dataQueue:
                novesDades.append(dataQueue.popleft())

        currentTime = time.time()
        
        # Processem cada paquet de dades
        for paquet in novesDades:
            parts = paquet.split(",")
            if len(parts) == 5:
                try:
                    valRespiratoria, valCardiaca, valSNS, valSNP, estres = map(float, parts)
                    timeStamp = currentTime - startTime
                    
                    dataCardiaca.append(valCardiaca)
                    dataRespiratoria.append(valRespiratoria)
                    timeStamps.append(timeStamp)
                    if valSNS != 0 and valSNP != 0:
                        dataSistemaNervios[0], dataSistemaNervios[1] = valSNS, valSNP
                        nivellEstres = estres
                except ValueError:
                    continue

        # Actualitzem gràfics amb temps
        if timeStamps:
            currentTimeElapsed = timeStamps[-1]
            timeWindowStartCardiaca = max(0, currentTimeElapsed - windowDurationCardiaca)
            timeWindowStartRespiratoria = max(0, currentTimeElapsed - windowDurationRespiratoria)
            
            # Actualitzem dades i límits
            lineCardiaca.set_data(timeStamps, dataCardiaca)
            lineRespiratoria.set_data(timeStamps, dataRespiratoria)
            
            axs[0, 0].set_xlim(timeWindowStartCardiaca, 
                              max(windowDurationCardiaca, currentTimeElapsed))
            axs[1, 0].set_xlim(timeWindowStartRespiratoria, 
                              max(windowDurationRespiratoria, currentTimeElapsed))

        # Actualitzem gràfic sistema nerviós
        for rect, val in zip(barSistemaNervios.patches, dataSistemaNervios):
            rect.set_height(val)

        # Actualitzem display d'estrès
        color = 'green' if nivellEstres < 50 else 'orange' if nivellEstres < 75 else 'red'
        textDisplay.set_color(color)
        textDisplay.set_text(f"{nivellEstres:.1f}%")

        plt.pause(0.05)

except KeyboardInterrupt:
    print("Programa aturat per teclat.")
    stopFlag = True

# Esperem que acabi el fil BLE
bleThread.join()
print("BLE desconnectat. Fi del programa.")
