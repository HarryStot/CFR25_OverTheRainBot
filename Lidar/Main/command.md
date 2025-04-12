# Liste des commandes à exécuter

Voici la liste des commandes nécessaires pour faire fonctionner votre système de détection d'obstacles avec LIDAR:

## 1. Créer un pseudo-terminal pour simuler le port série

```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

Cette commande affichera deux ports (ex: `/dev/pts/3` et `/dev/pts/4`). Notez ces ports.

## 2. Lancer le programme principal

```bash
python main.py
```

Assurez-vous d'avoir ajusté le port série dans le code si `/dev/pts/3` n'est pas le bon port.

## 3. Envoyer des commandes de position

Dans un autre terminal, envoyez des positions au format `POS,x,y,orientation,target_x,target_y`:

```bash
echo "POS,150,90,0,200,140" > /dev/pts/4
```

Remplacez `/dev/pts/4` par le deuxième port généré par socat.

## 4. Autres commandes de position à tester

```bash
echo "POS,10,10,0,300,300" > /dev/pts/4
echo "POS,100,100,45,200,200" > /dev/pts/4
echo "POS,200,150,90,300,150" > /dev/pts/4
```

## 5. Arrêter le programme

Utilisez `Ctrl+C` dans le terminal où main.py s'exécute.