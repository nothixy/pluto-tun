# 

Projet réseaux radio TPS IR RIO 2026 - Macéo TULOUP - Valentin FOULON

## Domaine d'application

Ce code est fait pour tourner sur un système Linux (testé sur Archlinux AMD64, Ubuntu 25.10 AArch64) ou macOS (non testé).

## Prérequis

Certaines parties du code nécessitent des features non incluses dans la branche stable de cargo, il faut donc changer la toolchain pour la version nightly
```sh
rustup default nightly
```
ou bien utiliser l'option `+nightly` à chaque commande cargo, par exemple
```sh
cargo +nightly build
```

## Installation des dépendances / compilation

L'installation des dépendances se fera automatiquement lors de la compilation.

Pour compiler l'exécutable dans le fichier `./target/debug/PlutoTUN`,
```sh
cargo build
```

Pour compiler l'exécutable au même endroit et le lancer par la suite,
```sh
cargo run
```

Enfin, l'interface réseau virtuelle TUN sur Linux demande des permissions. Deux possibilités s'offrent à vous : lancer le programme en tant que root
```sh
sudo ./target/debug/PlutoTUN
```
ou bien ajouter la capability `CAP_NET_ADMIN` sur l'exécutable pour ne plus avoir à utiliser sudo par la suite
```sh
sudo setcap CAP_NET_ADMIN=eip ./target/debug/PlutoTUN
```

Note : à la prochaine compilation avec `cargo build` ou `cargo run`, il faudra à nouveau lancer la commande setcap.
