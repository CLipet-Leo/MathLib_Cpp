#include "Test.h"

#include "MathLib.h"
#include "JsonConverter.h"

#include <fstream>

#define FILE_PATH "../data.json"

void testProdMat()
{
    Matrix m1(2, 3);
    m1[0][0] = 1;
    m1[0][1] = 2;
    m1[0][2] = 3;
    m1[1][0] = 4;
    m1[1][1] = 5;
    m1[1][2] = 6;
    MathLib::printMatrix(m1);
    Matrix m2(3, 2);
    m2[0][0] = 7;
    m2[0][1] = 8;
    m2[1][0] = 9;
    m2[1][1] = 10;
    m2[2][0] = 11;
    m2[2][1] = 12;
    MathLib::printMatrix(m2);
    Matrix m3 = m1 * m2;
    MathLib::printMatrix(m3);
}

void testInversedMatrix()
{
    Matrix m(3, 3);
    m[0][0] = 1;
    m[0][1] = -1;
    m[0][2] = 2;
    m[1][0] = 1;
    m[1][1] = 6;
    m[1][2] = 1;
    m[2][0] = 2;
    m[2][1] = 0;
    m[2][2] = -1;
    /*
    1 -1  2
    1  6  1
    2  0 -1
    */
    MathLib::printMatrix(m, "Base Matrix :");

    std::cout << "Determinant matrice : " << Matrix::deter(m) << '\n';

    Matrix comatrix = Matrix::com(m);
    MathLib::printMatrix(comatrix, "Comatrix :");

    Matrix transposed = Matrix::tran(m);
    MathLib::printMatrix(transposed, "Transposed :");

    Matrix inversed = Matrix::inverse(m);
    MathLib::printMatrix(inversed, "Inversed :");
}

void testTranslation()
{
    float m = 0.1f;
    float h = 0.1f;
    FVector3 F = FVector3(2.f, 0.f, 0.f);
    FVector3 G = FVector3(0.f, 0.f, 0.f);
    FVector3 v = FVector3(1.f, 1.f, 1.f);
    std::cout << "Before translation \n";
    std::cout << "F: " << F.ToString() << '\n';
    std::cout << "G: " << G.ToString() << '\n';
    std::cout << "v: " << v.ToString() << '\n';
    DoubleVector3 result = MathLib::translation(m, h, F, G, v);
    result.print();
}

void testRotation()
{
    float h = 0.1f;
    const std::vector<FVector3> F{FVector3(2.f, 0.f, 0.f), FVector3(0.f, 3.f, 0.f), FVector3(1.f, 1.f, 1.f)};
    const std::vector<FVector3> A{FVector3(1.f, 2.f, 0.f), FVector3(-1.f, -2.f, 1.f), FVector3(0.f, 1.f, -1.f)};
    const auto G = FVector3(0.f, 0.f, 0.f);
    Matrix I(3, 3);
    /*
    2 0 0
    0 3 0
    0 0 4
    */
    I[0][0] = 2;
    I[0][1] = 0;
    I[0][2] = 0;
    I[1][0] = 0;
    I[1][1] = 3;
    I[1][2] = 0;
    I[2][0] = 0;
    I[2][1] = 0;
    I[2][2] = 4;
    const auto teta = FVector3(0.1f, 0.2f, 0.3f);
    const auto tetap = FVector3(0.05f, -0.1f, 0.2f);

    const DoubleVector3 result = MathLib::rotation(h, F, A, G, I, teta, tetap);
    result.print();
}

void testInertia()
{
    const std::vector<FVector3> L = { FVector3(1.f, 2.f, 3.f), FVector3(4.f, 5.f, 6.f), FVector3(7.f, 8.f, 9.f)};
    const auto G = MathLib::centre_inert(L);
    std::cout << "G: " << G.ToString() << '\n';

    constexpr float m = 9.f;
    const auto I = MathLib::matrice_inert(L, m);
    MathLib::printMatrix(I, "Inertia matrix :");

    const auto O = FVector3(0.f, 0.f, 0.f);
    const auto A = FVector3(4.f, 5.f, 6.f);
    const auto IAW = MathLib::deplace_matrix(I, m, O, A);
    MathLib::printMatrix(IAW, "Replaced inertia matrix :");
}

void testPaveDroit()
{
    const FVector3 A0(0.f, 0.f, 0.f);
    constexpr unsigned int n = 50;
    constexpr float a = 3.f;
    constexpr float b = 3.f;
    constexpr float c = 3.f;
    const Matrix result = MathLib::pave_plein(n, a, b, c, A0);
    std::ofstream file(FILE_PATH);
    file << std::setfill(' ') << std::setw(2) << JsonConverter::MatrixToJson(result);
    file.close();
    //MathLib::printMatrix(result, "Pave droit :");
}

void testFactorielSinusCosinus()
{
    constexpr float x = 0.1f;
    constexpr int n = 5;

    std::cout << "factoriel(" << n << ") = " << MathLib::factoriel(n) << " (vs std::tgammaf(" << n + 1 << ") = " << std::tgammaf(n + 1) << ")\n";
    std::cout << "cos(" << x << ") = " << MathLib::cosinus(x, n) << " (vs std::cos(x) = " << std::cos(x) << ")\n";
    std::cout << "sin(" << x << ") = " << MathLib::sinus(x, n) << " (vs std::sin(x) = " << std::sin(x) << ")\n";
}

void testCercle()
{
    const FVector3 A0(0.f, 0.f, 0.f);
    const Matrix result = MathLib::cercle_plein(1.f, A0);
    std::ofstream file(FILE_PATH);
    file << std::setfill(' ') << std::setw(2) << JsonConverter::MatrixToJson(result);
    file.close();
    MathLib::printMatrix(result, "Cercle :");
}

void testCylindre()
{
    const FVector3 A0(0.f, 0.f, 0.f);
    const Matrix result = MathLib::cylindre_plein(2.f, 4.f, A0);
	std::ofstream file(FILE_PATH);
	file << std::setfill(' ') << std::setw(2) << JsonConverter::MatrixToJson(result);
	file.close();
    MathLib::printMatrix(result, "Cylindre :");
}

void testMouvement()
{
   // Paramètres du cylindre
    float R = 1.0f;
    float height = 4.0f;
    FVector3 base(0, 0, 0);

    // Création d'un cylindre plein
    Matrix cylindre = MathLib::cylindre_plein(R, height, base);

    // Extraction des points du cylindre dans un vecteur pour le calcul du centre et de la matrice d'inertie
    std::vector<FVector3> points;
    for (int i = 0; i < cylindre.getCols(); ++i)
        points.push_back(FVector3(cylindre[0][i], cylindre[1][i], cylindre[2][i]));

    // Calcul du centre d'inertie et de la matrice d'inertie (pour une masse totale donnée)
    float m = 10.0f;
    FVector3 centreInertie = MathLib::centre_inert(points);
    Matrix inertia = MathLib::matrice_inert(points, m);

    // Conditions initiales (au repos)
    FVector3 vitesseLineaire(0, 0, 0);
    FVector3 anglesInitiaux(0, 0, 0);
    FVector3 vitesseAngulaire(0, 0, 0);

    // Définition des forces :
    // - Gravité appliquée au centre (translation) : ici, on suppose la gravité dirigée vers le bas sur l'axe Z.
    FVector3 forceGravite(0, 0, -9.81f * m);

    // - Force horizontale appliquée sur le point le plus haut pour induire une rotation.
    // On cherche le point dont la coordonnée Z est maximale dans le cylindre.
    FVector3 pointHaut = points[0];
    for (const auto &p : points)
        if (p.getZ() > pointHaut.getZ())
            pointHaut = p;
    // Force horizontale (par exemple, selon l'axe X) appliquée sur ce point
    FVector3 forceExtra(50, 0, 0);

    // Construction des listes de forces et points d'application (les deux forces sont dans des sous-listes distinctes)
    std::vector<std::vector<FVector3>> forces;
    std::vector<std::vector<FVector3>> pointsApplication;
    // Gravité appliquée au centre d'inertie (n'affecte que la translation)
    forces.push_back({ forceGravite });
    pointsApplication.push_back({ centreInertie });
    // Force supplémentaire appliquée en haut (génère un moment pour la rotation)
    forces.push_back({ forceExtra });
    pointsApplication.push_back({ pointHaut });

    // Choix du pas de temps
    float dt = 0.1f;
    float T = 1.f;
    int n = 5;
    
    // Appel de la fonction de mouvement qui met à jour translation et rotation
    std::vector<Matrix> results = MathLib::trace_mouvements(cylindre, m, inertia, centreInertie, vitesseLineaire,
                                                 anglesInitiaux, vitesseAngulaire, forces, pointsApplication, dt, T, n);

    // Sauvegarde du résultat
    std::ofstream file(FILE_PATH);
    file << std::setfill(' ') << std::setw(2) << JsonConverter::MatrixToJson(results[3]);
    file.close();
}
