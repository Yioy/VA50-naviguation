# Système de navigation sans carte de véhicule autonome

Dans le cadre de notre projet de VA50 à l’UTBM, nous avons développé un système de navigation pour un véhicule autonome, permettant de s’affranchir d’une carte à haute résolution, et cherchant à être le plus autonome possible. Actuellement, certaines opérations nécessitent encore une cartographie et un positionnement grand public (haute résolution non nécessaire)

## Dépendances

Ce projet nécessite Ubuntu 18.04 avec ROS Melodic ou Ubuntu 20.04 avec ROS Noetic, Python 3.6+ (même avec Melodic), et catkin_tools (`apt install python3-catkin-tools`). Les modules Python nécessaires sont définis par `requirements.txt` et sont automatiquement installés à l’étape `./build_circulation.sh init`.

## Installation

La compilation et l’installation du projet se base sur un script shell `build_circulation.sh` qui installe automatiquement les dépendances et s’occupe des différences entre ROS Melodic et Noetic.

- Clonez le dépôt, ou copiez son contenu dans un workspace Catkin existant. Le projet utilise catkin_tools pour son installation, un workspace déjà compilé par catkin_make n’est pas compatible et devra être réinitialisé auparavant. Un workspace Melodic qui n’a pas été paramétré pour Python 3 devra aussi être réinitialisé avant usage (supprimer `build/` et `devel/`), car Catkin ne supporte pas le changement des paramètres additionnels de CMake après l’initialisation du workspace.
- Lancez la commande `./build_circulation.sh init` pour initialiser le workspace si nécessaire et installer les dépendances
- Lancez la commande `./build_circulation.sh build` pour compiler les packages
- Lancez la commande `./build_circulation.sh cython` pour compiler et installer les modules Cython (qui semble malheureusement incompatible avec le système de build de Catkin)

Cette dernière doit être relancée à chaque changement dans les modules Cython

## Contenu du projet

Le projet comprend 5 packages ROS :

- `transformtrack` gère le déplacement du véhicule et les changements de repère
- `circulation` gère la construction de trajectoire et les transformations
- `control` s’occupe du contrôle du véhicule
- `trafficsigns` détecte la signalisation verticale
- `direction` permet d’entrer directement des directions pour les intersections en console

## Exécution

La méthode la plus pratique pour lancer les nodes est la suivante. Cela lance les nodes désirés. Par défaut, seul transformtrack est lancé, dans le mode simulation, voir [utac.launch](src/bringup/launch/utac.launch).
```bash
roslaunch bringup utac.launch
```
Il est aussi possible de spécifier des arguments: pour lancer des nodes supplémentaires ou des fonctionnalités supplémentaires. Exemple:
```bash
roslaunch bringup utac.launch version:=Real launch_circulation:=true launch_trafficsigns:=true signs:=true
```
Remarque: pour l'instant, le roslaunch global ne lance pas le contrôle. 

Si vous avez besoin de lancer séparément les nodes, vous pouvez le faire de la façon suivante:

- Service de transformation (nécessaire pour tout le reste) : `rosrun transformtrack transformtrack_node`
- Construction de trajectoire : `roslaunch bringup circulation.launch`
- Détection des panneaux : `roslaunch bringup distance_extractor.launch` (et dans ce cas les paramètres du roslaunch global s'appliquent)
- Contrôle du véhicule : `rosrun control control.py src/bringup/config/circulation4-2.yml`
- Interface de test des directions en intersection : `rosrun direction signTransmit.py`

Le projet utilise un même fichier de paramètres au format YAML :

- `circulation4.yml` pour la version du simulateur du 31/10/2022
- `circulation4-1.yml` pour la version du 27/12/2022
- `circulation4-2.yml` pour la version du 03/01/2023

Les changements de visibilité entre les différentes versions impliquent la modification de quelques informations a priori (région d’intérêt, quelques paramètres de logique floue, …)

Si le fichier de configuration est modifié, il est nécessaire de lancer `python3 genconfig.py <fichier de config> transformtrack` pour regénérer la config des nodes C++,
puis de les recompiler avec `./build_circulation.sh build`

## Autres

Le topic camera_info étant manquant dans certains rosbags, le script `sim_camera_info.bash` peut être utilisé pour le publier.
