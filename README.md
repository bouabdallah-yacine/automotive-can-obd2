# 🚗 Réseau CAN automobile + diagnostic OBD-II (ESP32 + FreeRTOS)

[![Tests](https://github.com/bouabdellah-yacine/automotive-can-obd2/actions/workflows/ci.yml/badge.svg)](https://github.com/bouabdellah-yacine/automotive-can-obd2/actions/workflows/ci.yml)

Une voiture moderne contient des dizaines de calculateurs qui se parlent sur un **bus CAN**. Ce projet
simule ce réseau avec 4 nœuds (moteur, ABS, tableau de bord, outil de diagnostic) qui échangent des
trames **au bit près** : CRC-15, bit stuffing, arbitrage, trames d'erreur et mise hors ligne (bus-off)
d'un calculateur défaillant. L'outil de diagnostic lit les données et les **codes défauts OBD-II**,
comme la valise d'un garagiste.

> ✅ Simulé sur **Wokwi** (VS Code). Le cœur CAN / OBD-II est du C portable testé sur PC.

## Le réseau

| Nœud | Émet | Rôle |
|---|---|---|
| ECU moteur | `0x0C0` toutes les 50 ms | régime, température, papillon, voyant moteur ; répond au diagnostic `0x7E8` |
| ECU ABS | `0x1A0` toutes les 50 ms | vitesse du véhicule, frein, ABS actif |
| Tableau de bord | — | affichage OLED, détecte la perte de communication avec le moteur |
| Outil de diagnostic | `0x7DF`, `0x7E0` | requêtes OBD-II (filtre d'acceptation : ne reçoit que `0x7E8`) |

## Ce que montre le projet

- **Trame CAN 2.0A au bit près** : SOF, ID 11 bits, DLC, données, **CRC-15** (polynôme 0x4599),
  **bit stuffing**, ACK, EOF. La commande `bits` affiche la dernière trame telle qu'elle passe sur le câble.
- **Arbitrage non destructif** : bit dominant 0 contre récessif 1, l'ID le plus petit gagne.
- **Gestion des erreurs** : parasite détecté, trame d'erreur, réémission automatique ; compteurs
  **TEC / REC**, états *actif → passif → bus-off* : un calculateur défaillant se déconnecte tout seul.
- **OBD-II (SAE J1979)** : mode 01 (régime, vitesse, température, papillon), mode 03/04 (lire / effacer
  les codes défauts **P0217**, **P0480**), mode 09 (VIN).
- **ISO-TP (ISO 15765-2)** : le VIN (20 octets) est découpé en *First Frame* + contrôle de flux +
  *Consecutive Frames*.
- **FreeRTOS** : un calculateur par tâche, bus protégé par un mutex.
- **Dashboard web servi par l'ESP32** : compteur de vitesse et compte-tours, voyants, **espion du bus en
  direct** (comme un analyseur CAN), compteurs d'erreurs de chaque calculateur, boutons de diagnostic OBD-II.

## Résultats des tests (`test/test_can.c`, 13 tests)

- Erreur d'un bit injectée à **chacune des 106 positions** d'une trame : **106 / 106 détectées**.
- Erreurs doubles : **392 / 392 détectées**.
- Arbitrage, filtres, réémission, passage en bus-off (TEC ≥ 256), codes défauts, OBD-II, ISO-TP.

## Lancer la démo (Wokwi dans VS Code)

1. Ouvre ce dossier dans VS Code → PlatformIO **Build** → **F1 › Wokwi: Start Simulator**.
2. Ouvre **http://localhost:8181** : le dashboard (compteurs, voyants, trames en direct, diagnostic).
   Tourne le potentiomètre (**accélérateur**) ou la pédale de la page web : le régime et la vitesse montent.
3. Dans le moniteur série, tape :
   - `rpm`, `temp`, `vitesse`, `vin` : requêtes OBD-II, avec les trames brutes ;
   - `sniff` : espionne tout le trafic du bus, `bits` : la dernière trame au bit près ;
   - `stats` : charge du bus, erreurs, compteurs TEC/REC, dernier arbitrage.
4. Bouton **jaune « Panne ventilo »** puis accélère : la température dépasse 110 °C, le voyant
   **CHECK ENGINE** s'allume. Tape `dtc` : **P0480** et **P0217**. Puis `clear` pour les effacer.
5. Tape `error 40` : parasites sur les trames du moteur, son TEC monte jusqu'au **BUS-OFF**, et le tableau de bord
   affiche **PERTE COM MOTEUR U0100**. Le moteur revient sur le réseau 3 s plus tard.
6. Bouton **rouge « Frein »** à plus de 30 km/h : le voyant **ABS** clignote.

> Si la redirection de port ne fonctionne pas dans ta version de Wokwi, tout le reste marche :
> écran OLED, voyants et commandes dans le moniteur série.

## Tests

```bash
gcc -O2 -Wall -Wextra -Isrc -o t test/test_can.c src/can.c src/can_bus.c src/obd.c && ./t
```

## Licence

© 2026 Yacine — tous droits réservés. Code publié pour consultation uniquement (voir [`LICENSE`](LICENSE)).
