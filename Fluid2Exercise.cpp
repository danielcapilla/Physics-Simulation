#include "Scene.h"

#include "Numeric/PCGSolver.h"

namespace asa
{
namespace
{
////////////////////////////////////////////////
// Add any reusable classes or functions HERE //
////////////////////////////////////////////////
struct Emitter {
    Vector2 minPos;
    Vector2 maxPos;
    Vector3 ink;
    Vector2 velocity;
};
}  // namespace

// advection
void Fluid2::fluidAdvection(const float dt)
{
    // Ink Advection
    // Copy Ink to InkRBG0
    Array2<Vector3> inkRGB0(inkRGB);
    Index2 inkRGBSize = inkRGB.getSize();
    // Foreach cell/ij in InkRGB0
    for (int i = 0; i < inkRGBSize.x; i++) {
        for (int j = 0; j < inkRGBSize.y; j++) {
            // Get the cell center position
            Index2 id = Index2(i, j);
            Vector2 xt = grid.getCellPos(id);
            // Get the velocity
            float iNext = floor(clamp(i + 1, 0, inkRGBSize.x - 1));
            float jNext = floor(clamp(j + 1, 0, inkRGBSize.y - 1));
            float vx = (velocityX[id] + velocityX[Index2(iNext, j)]) * 0.5f;
            float vy = (velocityY[id] + velocityY[Index2(i, jNext)]) * 0.5f;
            // Calculate the new position
            Vector2 xt_1 = xt - dt * Vector2(vx, vy);
            // Get the index from the cell center position
            Vector2 i_xt_1 = grid.getCellIndex(xt_1);
            // Clamp to safe domain
            float clampPrevX = floor(clamp(i_xt_1.x, 0, inkRGBSize.x - 1));
            float clampX = floor(clamp(i_xt_1.x + 1, 0, inkRGBSize.x - 1));
            float clampPrevY = floor(clamp(i_xt_1.y, 0, inkRGBSize.y - 1));
            float clampY = floor(clamp(i_xt_1.y + 1, 0, inkRGBSize.y - 1));
            // Bilerp
            Vector3 aa = inkRGB0[Index2(clampPrevX, clampPrevY)];
            Vector3 ba = inkRGB0[Index2(clampX, clampPrevY)];
            Vector3 ab = inkRGB0[Index2(clampPrevX, clampY)];
            Vector3 bb = inkRGB0[Index2(clampX, clampY)];
            float t = i_xt_1.x - floor(i_xt_1.x);
            float s = i_xt_1.y - floor(i_xt_1.y);
            Vector3 bilerpInk = bilerp(aa, ba, ab, bb, t, s);
            // Set the new diag
            inkRGB[id] = bilerpInk;
        }
    }

    // Velocity advection
    Array2<float> velocityX0(velocityX);
    Array2<float> velocityY0(velocityY);
    Index2 sizeVx = velocityX.getSize();
    Index2 sizeVy = velocityY.getSize();
    //Vector2 h = grid.getDx();  // (dx, dy)
    //const float eps = 1e-4f;
    //// Velocity X
    //for (int i = 0; i < sizeVx.x; i++) {
    //    for (int j = 0; j < sizeVx.y; j++) {
    //        Index2 id = Index2(i, j);

    //        // u en la cara X (i,j)
    //        float u = velocityX0[id];
    //        Vector2 facePosition = grid.getFacePosX(id);
    //        // v alrededor de esa cara-X: (i-1,j), (i,j), (i-1,j+1), (i,j+1)
    //        int iYL = clamp(i - 1, 0, sizeVy.x - 1);
    //        int iYR = clamp(i, 0, sizeVy.x - 1);
    //        int jYB = clamp(j, 0, sizeVy.y - 1);
    //        int jYT = clamp(j + 1, 0, sizeVy.y - 1);
    //        float v0 = velocityY0[Index2(iYL, jYB)];
    //        float v1 = velocityY0[Index2(iYR, jYB)];
    //        float v2 = velocityY0[Index2(iYL, jYT)];
    //        float v3 = velocityY0[Index2(iYR, jYT)];
    //        float v = 0.25f * (v0 + v1 + v2 + v3);

    //        // Retrotrazado en espacio de índices de u
    //        Vector2 prevIdx = Vector2((float)i, (float)j) - dt * Vector2(u / h.x, v / h.y);

    //        // Clamp continuo a un rango seguro para garantizar 2 muestras válidas
    //        // Nota: usar (nx-1-eps) fuerza i0 <= nx-2 y i1 = i0+1 <= nx-1

    //        prevIdx.x = clamp(prevIdx.x, 0.0f, (float)sizeVx.x - 1.0f - eps);
    //        prevIdx.y = clamp(prevIdx.y, 0.0f, (float)sizeVx.y - 1.0f - eps);

    //        int i0 = (int)floor(prevIdx.x);
    //        int j0 = (int)floor(prevIdx.y);
    //        int i1 = i0 + 1;
    //        int j1 = j0 + 1;

    //        float aa = velocityX0[Index2(i0, j0)];
    //        float ba = velocityX0[Index2(i1, j0)];
    //        float ab = velocityX0[Index2(i0, j1)];
    //        float bb = velocityX0[Index2(i1, j1)];

    //        float t = prevIdx.x - (float)i0;  // 0 <= t < 1
    //        float s = prevIdx.y - (float)j0;  // 0 <= s < 1

    //        velocityX[id] = bilerp(aa, ba, ab, bb, t, s);
    //    }
    //}
    // Velocity X
    for (int i = 0; i < sizeVx.x; i++) {
        for (int j = 0; j < sizeVx.y; j++) {
            Index2 id = Index2(i, j);

            Vector2 facePosition = grid.getFacePosX(id);

            int iY = clamp(i, 0, sizeVy.x - 1); // iYRight
            int iYPrev = clamp(i - 1, 0, sizeVy.x - 1);  // iYLeft
            //int iYNext = clamp(i + 1, 0, sizeVy.x - 1);
            int jY = clamp(j, 0, sizeVy.y - 1);  // jYBottom
            //int jYPrev = clamp(j - 1, 0, sizeVy.y - 1);
            int jYNext = clamp(j + 1, 0, sizeVy.y - 1);  // jYTop

            float v0 = velocityY0[Index2(iYPrev, jY)];
            float v1 = velocityY0[Index2(iY, jY)];
            float v2 = velocityY0[Index2(iYPrev, jYNext)];
            float v3 = velocityY0[Index2(iY, jYNext)];
            float u = velocityX0[id];
            float v = 0.25f * (v0 + v1 + v2 + v3);

            Vector2 prevPosition = facePosition - dt * Vector2(u, v);
            Vector2 prevId = grid.getCellIndex(prevPosition);

            float clampPrevX = floor(clamp(prevId.x, 0, sizeVx.x - 1));
            float clampX = floor(clamp(prevId.x + 1, 0, sizeVx.x - 1));
            float clampPrevY = floor(clamp(prevId.y, 0, sizeVx.y - 1));
            float clampY = floor(clamp(prevId.y + 1, 0, sizeVx.y - 1));

            float aa = velocityX0[Index2(clampPrevX, clampPrevY)];
            float ba = velocityX0[Index2(clampX, clampPrevY)];
            float ab = velocityX0[Index2(clampPrevX, clampY)];
            float bb = velocityX0[Index2(clampX, clampY)];

            float t = prevId.x - floor(prevId.x);
            float s = prevId.y - floor(prevId.y);
            float bilerpVelX = bilerp(aa, ba, ab, bb, t, s);
            velocityX[id] = bilerpVelX;
        }
    }

    // Velocity Y
    for (int i = 0; i < sizeVy.x; i++) {
        for (int j = 0; j < sizeVy.y; j++) {
            Index2 id = Index2(i, j);

            Vector2 facePosition = grid.getFacePosY(id);

            int iX = clamp(i, 0, sizeVx.x - 1);
            int iXNext = clamp(i + 1, 0, sizeVx.x - 1);
            int iXPrev = clamp(i - 1, 0, sizeVx.x - 1);
            int jX = clamp(j, 0, sizeVx.y - 1);
            int jXNext = clamp(j + 1, 0, sizeVx.y - 1);
            int jXPrev = clamp(j - 1, 0, sizeVx.y - 1);

            float u0 = velocityX0[Index2(iX, jXPrev)];
            float u1 = velocityX0[Index2(iX, jX)];
            float u2 = velocityX0[Index2(iXNext, jXPrev)];
            float u3 = velocityX0[Index2(iXNext, jX)];
            float v = velocityY0[id];
            float u = (u0 + u1 + u2 + u3) * 0.25f;

            Vector2 prevPosition = facePosition - dt * Vector2(u, v);
            Vector2 prevId = grid.getCellIndex(prevPosition);

            float clampPrevX = floor(clamp(prevId.x, 0, sizeVy.x - 1));
            float clampX = floor(clamp(prevId.x + 1, 0, sizeVy.x - 1));
            float clampPrevY = floor(clamp(prevId.y, 0, sizeVy.y - 1));
            float clampY = floor(clamp(prevId.y + 1, 0, sizeVy.y - 1));

            float aa = velocityY0[Index2(clampPrevX, clampPrevY)];
            float ba = velocityY0[Index2(clampX, clampPrevY)];
            float ab = velocityY0[Index2(clampPrevX, clampY)];
            float bb = velocityY0[Index2(clampX, clampY)];

            float t = prevId.x - floor(prevId.x);
            float s = prevId.y - floor(prevId.y);
            float bilerpVelY = bilerp(aa, ba, ab, bb, t, s);
            velocityY[id] = bilerpVelY;
        }
    }
}

void Fluid2::fluidEmission()
{
    if (Scene::testcase >= Scene::SMOKE) {
        // Emitters contribution HERE
        /*
        * Emisor 1 Domain = (-0.1, -1.9), (0.1, -1.75);
        Ink = (1,1,0)
        Velocity = (0,8)

        Emisor 2 Domain = (-0.2, -1.9),(-0.1, -1.75);
        Ink = (1, 0, 1)Velocity = (0, 8)

        Emisor 3 Domain = (0.1, -1.9),
        (0.2, -1.75);
        Ink = (0, 0, 1)Velocity = (0, 8)
        */
        std::vector<Emitter> emitters = {
            {Vector2(-0.1f, -1.9f), Vector2(0.1f, -1.75f), Vector3(1.0f, 1.0f, 0.0f), Vector2(0.0f, 8.0f)},
            {Vector2(-0.2f, -1.9f), Vector2(-0.1f, -1.75f), Vector3(1.0f, 0.0f, 1.0f), Vector2(0.0f, 8.0f)},
            {Vector2(0.1f, -1.9f), Vector2(0.2f, -1.75f), Vector3(0.0f, 0.8f, 1.0f), Vector2(0.0f, 8.0f)}};

        for (int i = 0; i < inkRGB.getSize().x; i++) {
            for (int j = 0; j < inkRGB.getSize().y; j++) {
                Index2 id = Index2(i, j);
                Vector2 pos = grid.getCellPos(id);  


                for (const auto &emitter : emitters) {
                    if (pos.x >= emitter.minPos.x && pos.x <= emitter.maxPos.x && pos.y >= emitter.minPos.y &&
                        pos.y <= emitter.maxPos.y) {
                        inkRGB[id] = emitter.ink;            
                        velocityX[id] = emitter.velocity.x;  
                        velocityY[id] = emitter.velocity.y;  
                    }
                }
            }
        } 
    }
}

void Fluid2::fluidVolumeForces(const float dt)
{
    if (Scene::testcase >= Scene::SMOKE) {
        // Gravity term HERE

        // Aplicar gravedad a todas las celdas de la velocidad en el eje Y
        for (int i = 0; i < velocityY.getSize().x; i++) {
            for (int j = 0; j < velocityY.getSize().y; j++) {
                Index2 id = Index2(i, j);
                velocityY[id] += Scene::kGravity * dt;
            }
        }
    }
}

void Fluid2::fluidViscosity(const float dt)
{
    if (Scene::testcase >= Scene::SMOKE) {
        // Viscosity term HERE
        // DIFUSION
        Array2<float> velocityX0 = velocityX;
        Array2<float> velocityY0 = velocityY;

        Index2 sizeVx = velocityX.getSize();
        Index2 sizeVy = velocityY.getSize();
        float Dx2 = pow(grid.getDx().x, 2);
        float Dy2 = pow(grid.getDx().y, 2);

        // Iterar sobre cada velocidad para aplicar la viscosidad
        for (int i = 0; i < sizeVx.x; i++) {
            for (int j = 0; j < sizeVx.y; j++) {
                Index2 id(i, j);
                Index2 uiNext(clamp(i + 1, 0, sizeVx.x - 1), j);
                Index2 uiPrev(clamp(i - 1, 0, sizeVx.x - 1), j);
                Index2 ujNext(i, clamp(j + 1, 0, sizeVx.y - 1));
                Index2 ujPrev(i, clamp(j - 1, 0, sizeVx.y - 1));
                // Integrador explicito
                velocityX[id] = velocityX0[id] + dt * Scene::kViscosity / Scene::kDensity *
                                         ((velocityX0[uiNext] - 2 * velocityX0[id] + velocityX0[uiPrev]) / Dx2 +
                                          (velocityX0[ujNext] - 2 * velocityX0[id] + velocityX0[ujPrev]) / Dy2);
            }
        }
        for (int i = 0; i < sizeVy.x; i++)
        {
            for (int j = 0; j < sizeVy.y; j++)
            {
                Index2 id(i, j);
                Index2 uiNext(clamp(i + 1, 0, sizeVy.x - 1), j);
                Index2 uiPrev(clamp(i - 1, 0, sizeVy.x - 1), j);
                Index2 ujNext(i, clamp(j + 1, 0, sizeVy.y - 1));
                Index2 ujPrev(i, clamp(j - 1, 0, sizeVy.y - 1));
                // Integrador explicito
                velocityY[id] = velocityY0[id] + dt * Scene::kViscosity / Scene::kDensity *
                                         ((velocityY0[uiNext] - 2 * velocityY0[id] + velocityY0[uiPrev]) / Dx2 +
                                          (velocityY0[ujNext] - 2 * velocityY0[id] + velocityY0[ujPrev]) / Dy2);
            }
        }
    }
}

void Fluid2::fluidPressureProjection(const float dt)
{
    if (Scene::testcase >= Scene::SMOKE) {
        // Incompressibility / Pressure term HERE

        // foreach velocityX in border (L,R)
        // velocityX[i,j] = 0
        // foreach velocityYin border (D,U)
        // velocityX¡Y[i,j] = 0
        // 
        // Dof = NcellsX x NcellsY
        //VectorDerecho<double> (Dof)
        // foreach cell
        //        Compute RHSmultiplied by constraints
        // Cada ecuancion (p.e 27), le corresponde una celda (p.e (2,1)) USAMOS GetLinearIndex
        // cellRHS = Compute velocity divergence mjultiplied by constants
        //        RHS[GetlinearIndex[i,j] = cellRHS (density*divergencia en cada celda)

        // SparseMatrix<double> A(NDof, 5)

        // foreach cell
        //      indexDiag = linearIndex(i,j)
        //      indexL = linearIndex(i-1,j)
        //      indexR = linearIndex(i+1,j)
        //      indexD = linearIndex(i,j-1)
        //      indexU = linearIndex(i,j+1)
        //      A(indexDiag, indexDiag) = -2/ Dx^2 - 2/Dy^2 
        //      fill offdiag terms   
        //      condiciones de contorno afecta a la frontera
        //      LoR -1/ Dx^2
        //      UoD -1/ Dy^2

        //  Vector<double> Incognitas(Dof)
        //  Init all Incognitas to 0

        //  Solve
        //  PCGSolver<double> solver
        // PCG set_solver_parameters 1e-5, 200
        //  residual, iterations
        //  solver.solve(A, RHS, Incognitas)

        // foreach NDofs
        //      pressure[Dof] = static_cast<float>incognitas[Dof]

        //  foreach VelocityX CUANDO NO ESTAMOS EN EL BORDE
        //    velocityX[i,j] -= gradConstant P wrt x ()
        //	  VelocityX[i,j] -= dt * (Incognitas[GetLinearIndex(i+1,j)] - Incognitas[GetLinearIndex(i,j)]) / Dx
        //  foreach VelocityY CUANDO NO ESTAMOS EN EL BORDE
        //    velocityY[i,j] -= gradConstant P wrt y ()
        //	  VelocityY[i,j] -= dt * (Incognitas[GetLinearIndex(i,j+1)] - Incognitas[GetLinearIndex(i,j)]) / Dy

        // ECUACIONES DE POISSON
        Index2 sizeVx = velocityX.getSize();
        Index2 sizeVy = velocityY.getSize();

        // Columnaxfila
        // Izquierda y derecha
        for (int i = 0; i < sizeVx.y; i++)
        {
            velocityX[Index2(0, i)] = 0.0f;
            velocityX[Index2(sizeVx.x - 1, i)] = 0.0f;
        }
        // Arriba y abajo
        for (int i = 0; i < sizeVy.x; i++) {
            velocityY[Index2(i, 0)] = 0.0f;
            velocityY[Index2(i, sizeVy.y - 1)] = 0.0f;
        }
        Index2 pressureSize = pressure.getSize();
        const int DoF = pressureSize.x * pressureSize.y;
        float Dx = grid.getDx().x;
        float Dy = grid.getDx().y;
        float Dx2 = pow(grid.getDx().x, 2);
        float Dy2 = pow(grid.getDx().y, 2);
        float constant = Scene::kDensity / dt ;
        std::vector<double> RHS(DoF);
        for (int i = 0; i < pressureSize.x; i++)
        {
            for (int j = 0; j < pressureSize.y; j++)
            {
                Index2 id(i, j);
                Index2 uiNext(clamp(i + 1, 0, sizeVx.x - 1), j);
                Index2 ujNext(i, clamp(j + 1, 0, sizeVy.y - 1));
                RHS[pressure.getLinearIndex(i, j)] = - constant * ((velocityX[uiNext] - velocityX[id]) / Dx +
                                                                 (velocityY[ujNext] - velocityY[id]) / Dy);
            }
        }

        SparseMatrix<double> A(DoF, 5); 

        for (int i = 0; i < pressureSize.x; ++i) {
            for (int j = 0; j < pressureSize.y; ++j) {
                int id = pressure.getLinearIndex(i, j);

                double diag = 2.0 / Dx2 + 2.0 / Dy2;
                // Condicion de contorno (Dirichlet)
                // Borde horizontal y vertical
                if (i == 0 || i == pressureSize.x - 1)
                    diag -= 1.0f / Dx2;
                if (j == 0 || j == pressureSize.y - 1)
                    diag -= 1.0f / Dy2;
                // -1 a los vecinos
                // Izquierdo
                // Vecinos
                if (i > 0)
                    A.add_to_element(id, pressure.getLinearIndex(i - 1, j), -1.0 / Dx2);
                if (i < pressureSize.x - 1)
                    A.add_to_element(id, pressure.getLinearIndex(i + 1, j), -1.0 / Dx2);
                if (j > 0)
                    A.add_to_element(id, pressure.getLinearIndex(i, j - 1), -1.0 / Dy2);
                if (j < pressureSize.y - 1)
                    A.add_to_element(id, pressure.getLinearIndex(i, j + 1), -1.0 / Dy2);

                A.set_element(id, id, diag);
            }
        }

        PCGSolver<double> solver;
        double residualOut;
        int iterationsOut;
        solver.set_solver_parameters(1e-5, 200);
        std::vector<double> incognitas(DoF, 0.0);
        solver.solve(A, RHS, incognitas, residualOut, iterationsOut);

        for (int i = 0; i < pressureSize.x; i++) {
            for (int j = 0; j < pressureSize.y; j++) {
                Index2 id(i, j);
                pressure[id] = static_cast<float>(incognitas[pressure.getLinearIndex(i, j)]);
            }
        }
        // FLUIDO INCOMPRESIBLE
        float gradConstant = dt / Scene::kDensity;
        for (int i = 1; i < sizeVx.x-1; i++) {
            for (int j = 0; j < sizeVx.y; j++) {
                Index2 id(i, j);
                velocityX[id] += -gradConstant * (pressure[id] -pressure[Index2(i - 1, j)]) / Dx;
            }
        }
        for (int i = 0; i < sizeVy.x; i++) {
            for (int j = 1; j < sizeVy.y-1; j++) {
                Index2 id(i, j);
                velocityY[id] += -gradConstant * (pressure[id] - pressure[Index2(i, j - 1)]) / Dy;
            }
        }

    }
}
}  // namespace asa
