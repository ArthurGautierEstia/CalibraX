# CalibraX Agent Rules

Ce fichier cadre les assistants IA qui modifient le projet.

## Objectif

Le code Python doit preparer une future migration vers C++.
Les choix de conception doivent donc privilegier:

- des types explicites
- des structures stables et lisibles
- peu de logique implicite
- peu de donnees faiblement typees

## Regles non negociables

- Ne pas utiliser `dict[str, Any]` ou un dictionnaire libre comme pseudo-objet metier.
- Ne pas utiliser `list[float]`, `tuple[float, float, float]` ou autres sequences brutes pour representer une donnee metier de taille fixe quand un type dedie existe ou doit exister.
- Si une structure metier de taille fixe apparait regulierement, creer un type dedie dans `models/types/`.
- Les APIs internes du domaine ne doivent pas accepter plusieurs representations du meme concept. Un concept metier = un type principal.
- Les conversions depuis/vers des listes, tuples ou dictionnaires doivent rester aux frontieres:
  - JSON
  - fichiers
  - signaux UI
  - export CSV
  - interop bibliotheques externes

## Typage attendu

- Utiliser des types explicites dans les signatures, attributs et retours.
- Preferer un objet dedie plutot qu'un tuple si chaque composante a un sens metier.
- Eviter `Any`, `object`, unions larges et structures heterogenes, sauf vrai besoin d'interop.
- Lorsqu'un type existe deja, l'utiliser partout au lieu de recreer une forme equivalente.

## Regles de conception

- Normaliser les donnees une seule fois a l'entree du systeme, puis travailler ensuite avec des types forts.
- Eviter les listes temporaires inutiles si l'objet possede deja les methodes necessaires (`copy()`, `to_list()`, `normalized()`, etc.).
- Garder des noms explicites, en particulier pour les unites: `_mm`, `_deg`, `_rad`, `_mps`, etc.
- Favoriser des objets metier simples, predictibles et proches d'un futur modele C++.

## Avant d'introduire un nouveau type primitif

Se demander dans cet ordre:

1. Un type dedie existe-t-il deja dans `models/types/` ?
2. Si non, la structure a-t-elle un sens metier stable ?
3. Si oui, faut-il creer un nouveau type dedie plutot qu'une `list[float]` ou un `tuple[...]` ?

## Reference

Les conventions detaillees sont documentees dans [docs/CODING_GUIDELINES.md](docs/CODING_GUIDELINES.md).
