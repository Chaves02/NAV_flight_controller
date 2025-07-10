import pandas as pd
import matplotlib.pyplot as plt

# Caminho para o ficheiro CSV
file_path = "Telemetria.csv"  # Altera este nome conforme necessário

# Carregar CSV com separador ';'
df = pd.read_csv(file_path, delimiter=';')

# Converter tempo de string para float
df['time_s'] = df['time_s'].str.replace(',', '.').astype(float)

# ----------- 1. Roll e Pitch -----------
plt.figure(figsize=(11.7, 4))
plt.plot(df['time_s'], df['roll'], label='Roll')
plt.plot(df['time_s'], df['pitch'], label='Pitch')
plt.title('Roll e Pitch ao longo do tempo')
plt.xlabel('Tempo (s)')
plt.ylabel('Ângulo (°)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("roll_pitch.png", dpi=300)
plt.close()

# ----------- 2. Velocidades angulares -----------
plt.figure(figsize=(11.7, 4))
plt.plot(df['time_s'], df['rate_roll'], label='Rate Roll')
plt.plot(df['time_s'], df['rate_pitch'], label='Rate Pitch')
plt.plot(df['time_s'], df['rate_yaw'], label='Rate Yaw')
plt.title('Velocidades angulares')
plt.xlabel('Tempo (s)')
plt.ylabel('Velocidade (°/s)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("angular_rates.png", dpi=300)
plt.close()

# ----------- 3. Altitude e Velocidade -----------
plt.figure(figsize=(11.7, 4))
plt.plot(df['time_s'], df['altitude'], label='Altitude')
plt.plot(df['time_s'], df['velocity'], label='Velocidade')
plt.title('Altitude e Velocidade')
plt.xlabel('Tempo (s)')
plt.ylabel('Unidades')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("altitude_velocity.png", dpi=300)
plt.close()

# ----------- 4. Potência dos Motores -----------
plt.figure(figsize=(11.7, 4))
plt.plot(df['time_s'], df['motor1'], label='Motor 1')
plt.plot(df['time_s'], df['motor2'], label='Motor 2')
plt.plot(df['time_s'], df['motor3'], label='Motor 3')
plt.plot(df['time_s'], df['motor4'], label='Motor 4')
plt.title('Potência dos Motores')
plt.xlabel('Tempo (s)')
plt.ylabel('PWM')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("motor_outputs.png", dpi=300)
plt.close()
