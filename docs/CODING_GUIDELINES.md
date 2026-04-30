# Coding Guidelines

## But du document

Ce document fixe les conventions de code pour CalibraX.
L'objectif principal est de garder un code Python facile a maintenir aujourd'hui, tout en preparant une migration future vers C++ avec un minimum de reinterpretation.

En pratique, cela signifie:

- modeliser clairement les donnees metier
- reduire les representations multiples d'un meme concept
- limiter les structures dynamiques et faiblement typees
- rendre les interfaces internes explicites et stables

## Principe central

Dans le code metier, un concept doit avoir une representation principale unique.

Exemples:

- une pose cartesienne 6D doit etre un `Pose6`
- un vecteur XYZ doit etre un `XYZ3`
- deux vecteurs tangents ne doivent pas devenir une `list[list[float]]` si un meilleur type existe

Les formats bruts restent acceptables uniquement aux frontieres techniques du systeme.

## Frontieres ou les formes brutes sont autorisees

Les listes, tuples et dictionnaires bruts sont toleres uniquement pour:

- la lecture/ecriture JSON
- l'import/export de fichiers
- l'interop avec Qt
- l'interop avec NumPy ou une lib externe
- les formats de transport temporaires clairement identifies

Regle importante:

- convertir au plus tot vers un type metier
- reconvertir au plus tard vers le format externe

Autrement dit, les structures brutes ne doivent pas traverser tout le coeur applicatif.

## Regles de typage

### 1. Pas de pseudo-objets dynamiques

A eviter:

```python
payload: dict[str, Any] = {
    "x": 1.0,
    "y": 2.0,
    "z": 3.0,
}
```

Preferer:

```python
position = XYZ3(1.0, 2.0, 3.0)
```

Ou, si la structure n'est pas geometrique mais reste metier:

- creer une classe dediee
- ou une `@dataclass` si c'est un simple objet de donnees bien defini

Un dictionnaire libre ne doit pas remplacer un objet metier.

### 2. Pas de sequence brute pour une structure fixe

A eviter:

- `list[float]` pour une pose 6D
- `tuple[float, float, float]` pour un vecteur 3D
- `list[float]` pour un jeu de 6 articulations si la structure a un vrai sens metier stable

Preferer:

- `Pose6`
- `XYZ3`
- un nouveau type dans `models/types/` si necessaire

### 3. Pas de doubles representations dans les APIs internes

A eviter:

```python
def build_target(target: Pose6 | Sequence[float] | None) -> None:
    ...
```

Preferer:

```python
def build_target(target: Pose6 | None) -> None:
    ...
```

Si la source externe fournit une liste, la conversion doit etre faite avant l'appel.

### 4. Eviter `Any`

`Any` doit rester exceptionnel.
Si un type n'est pas encore clair:

- soit il faut modeliser un vrai type
- soit la fonction se trouve trop pres d'une frontiere technique et doit rester localisee

## Regles de modelisation

### Types a creer quand il y a un gain de lisibilite

Si une forme revient souvent avec une semantique stable, creer un type dedie dans `models/types/`.

Exemples possibles selon l'evolution du projet:

- `Joint6`
- `JointLimits6`
- `CartesianTwist6`
- `CartesianAcceleration6`
- `BezierTangentPair`

Le critere n'est pas "est-ce qu'on peut techniquement mettre une liste ?".
Le critere est "est-ce que cela clarifie le domaine et facilite une future traduction vers C++ ?".

### Objets simples et predictibles

Les objets metier doivent:

- avoir des attributs explicites
- avoir des noms stables
- eviter les champs ambigus
- avoir des conversions explicites (`to_list()`, `from_values()`, `copy()`, etc.) seulement si utiles

### Unites explicites

Les unites doivent apparaitre dans les noms quand il y a un risque d'ambiguite.

Exemples:

- `distance_mm`
- `speed_mps`
- `angle_deg`
- `angle_rad`

## Regles d'implementation

### Convertir une fois

Une donnee brute doit etre convertie une seule fois a la frontiere, puis manipulee sous forme typée.

A eviter:

- reconstruire plusieurs fois `Pose6(*values[:6])`
- recreer plusieurs fois `XYZ3(values[0], values[1], values[2])`
- passer sans cesse d'un objet typé a une liste puis revenir a l'objet

### Eviter les structures temporaires inutiles

Si un type possede deja les bonnes methodes, les utiliser directement.

A eviter:

```python
XYZ3(-v.x, -v.y, -v.z).normalized().to_list()
```

Preferer, si le type le permet:

```python
(-v).normalized()
```

### Retours de fonctions

Pour les retours:

- preferer un objet dedie si le resultat a une vraie semantique
- utiliser un tuple seulement pour un regroupement simple, local et clairement lisible

Si un tuple commence a transporter une structure metier recurrente, il faut envisager un type dedie.

## Regles specifiques a la migration C++

Le code doit tendre vers un style facile a projeter en C++:

- types metier explicites
- peu d'heterogeneite
- peu de runtime implicite
- peu de dictionnaires fourre-tout
- peu de conversions silencieuses
- objets a responsabilite claire

Questions a se poser avant une modification:

1. Est-ce que cette structure aurait une classe ou un `struct` en C++ ?
2. Si oui, pourquoi ne pas deja la modeliser clairement en Python ?
3. Est-ce que cette signature masque un manque de modelisation ?

## Check-list avant de valider un changement

- Est-ce qu'un type metier existant pouvait etre reutilise ?
- Est-ce qu'une `list[float]` ou un `tuple[...]` remplace en fait un vrai concept metier ?
- Est-ce qu'un `dict` est utilise comme objet de domaine ?
- Est-ce que la conversion brut -> type metier est faite a la frontiere ?
- Est-ce que l'API interne accepte une seule representation claire du concept ?
- Est-ce que les noms et unites sont explicites ?

## Priorite en cas de doute

Quand plusieurs options sont possibles, preferer dans cet ordre:

1. clarte du modele metier
2. typage strict
3. facilite de migration vers C++
4. simplicite d'usage interne
5. compatibilite avec les formats externes

Les formats externes doivent s'adapter au modele interne, pas l'inverse.
