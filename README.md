# TUN interface via ADALM Pluto

Projet réseaux radio TPS IR RIO 2026 - Macéo TULOUP - Valentin FOULON

## Domaine d'application

Ce code est fait pour tourner sur un système Linux (testé sur Archlinux AMD64, Ubuntu 25.10 AArch64).
Il peut être adapté sur Mac, mais nécessite de comprendre la documentation sur utun.

## Prérequis

Certaines parties du code nécessitent des features non incluses dans la branche stable de cargo, il faut donc changer la toolchain pour la version nightly
```sh
rustup default nightly
```
ou bien utiliser l'option `+nightly` à chaque commande cargo, par exemple
```sh
cargo +nightly build
```

Ce programme nécessite d'utiliser deux cartes ADALM Pluto afin de réaliser des échanges. Deux PC sont nécessaires afin que Linux puisse envoyer des réponses via la bonne interface sans essayer de passer par le loopback.

## Installation des dépendances / compilation

L'installation des dépendances rust se fera automatiquement lors de la compilation. Il faut cependant avoir installé la bibliothèque IIO avant.

Pour compiler l'exécutable dans le fichier `./target/debug/PlutoTUN`,
```sh
cargo build
```

Pour compiler l'exécutable au même endroit et le lancer par la suite,
```sh
cargo run
```

Enfin, l'interface réseau virtuelle TUN sur Linux demande des permissions. Il faut donc lancer le programme en tant que root.

Pour lancer le premier pair :
```sh
sudo ./target/debug/PlutoTUN server
```

Et pour lancer le second :
```sh
sudo ./target/debug/PlutoTUN client
```
