#!/bin/bash

# Configura
INTERFAZ="enp3s0"
IP_LOCAL="192.168.1.100"
RED="/16"
TIEMPO_ESPERA=20

# 1. Limpia la configuración anterior
echo "[INFO] Limpiando IP previa de $INTERFAZ..."
sudo ip addr flush dev $INTERFAZ

# 2. Asigna IP genérica amplia
echo "[INFO] Asignando IP genérica $IP_LOCAL$RED..."
sudo ip addr add $IP_LOCAL$RED dev $INTERFAZ

# 3. Escanea tráfico con tcpdump durante X segundos
echo "[INFO] Escuchando tráfico en $INTERFAZ durante $TIEMPO_ESPERA segundos..."
echo "[INFO] Reinicia o reconecta las cámaras ahora."
sudo timeout $TIEMPO_ESPERA tcpdump -i $INTERFAZ -nn -q > /tmp/tcpdump_output.txt

# 4. Filtra IPs detectadas
echo -e "\n[INFO] Posibles cámaras detectadas:\n"

grep -oP 'IP \K([0-9]{1,3}\.){3}[0-9]{1,3}' /tmp/tcpdump_output.txt \
  | grep -v "$IP_LOCAL" \
  | sort -u \
  | while read ip; do
    BASE=$(echo $ip | cut -d '.' -f 1-3)
    echo "$ip    (sudo ip addr flush dev $INTERFAZ && sudo ip addr add $BASE.100/24 dev $INTERFAZ)"
done

