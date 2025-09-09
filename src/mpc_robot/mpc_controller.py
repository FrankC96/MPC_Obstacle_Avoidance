import os, sys
import numpy as np
import numpy.typing as npt
from scipy.optimize import minimize, NonlinearConstraint

from dataclasses import dataclass
from typing import Union, Callable, Optional


@dataclass
class Controller:
    name: str
    constr_method: str
    model: Callable[
        [npt.NDArray[np.float64], npt.NDArray[np.float64]], npt.NDArray[np.float64]
    ]
    nx: int
    nu: int
    x_guess: Union[float, int]
    u_guess: Union[float, int]
    n_pred: int
    dt: float
    Q: npt.NDArray[np.float64]
    R: npt.NDArray[np.float64]
    minimize_method: str
    verbose: bool = True

    def __post_init__(self):
        print(f"Initializing MPC controller with {self.dt} time-step.")
        self.X: Optional[npt.NDArray[np.float64]] = None

    def euler_step(self, x: npt.NDArray[np.float64], u: npt.NDArray[np.float64]):
        return x + self.dt * self.model(x, u)

    def RK4_step(
        self, x: npt.NDArray[np.float64], u: npt.NDArray[np.float64], f_noise=None
    ):
        f = self.model
        if f_noise:
            f = f_noise
        k1 = f(x, u)
        k2 = f(x + k1 * self.dt / 2, u)
        k3 = f(x + k2 * self.dt / 2, u)
        k4 = f(x + k3 * self.dt, u)

        return x + self.dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6

    def shooting_step(self, x, u):
        """Simulate one shooting interval"""
        return self.model(x, u)

    def objective(self, X: npt.NDArray[np.float64], xref: npt.NDArray[np.float64]):
        """Objective function: minimize control effort"""
        X = X.reshape([self.n_pred, self.nx + self.nu], order="F")
        X_state = X[:, : self.nx]
        X_ref = np.tile(xref, self.nx)

        U = X[:, -self.nu :]

        state_cost = np.sum(
            [
                ((X_state[i] - xref) @ self.Q) @ (X_state[i].T - xref.T)
                for i in range(len(X_state))
            ]
        )

        input_cost = np.sum([(U[i] @ self.R) @ U[i] for i in range(len(U))])

        total_cost = state_cost + input_cost
        return total_cost

    def constraints(self, X: npt.NDArray[np.float64], x0):
        """Constraints for direct multiple shooting"""
        X = X.reshape([self.n_pred, self.nx + self.nu], order="F")
        ceq = []

        if self.constr_method == "DMS":
            for i in range(self.n_pred - 1):
                X_next = self.model(X[i, : self.nx], X[i, -self.nu :])
                ceq.extend((X[i + 1, : self.nx] - X_next).tolist())

        elif self.constr_method == "COLL":
            for i in range(self.n_pred - 1):
                ceq.extend(
                    X[i + 1, : self.nx]
                    - X[i, : self.nx]
                    - (self.dt / 2)
                    * (
                        self.model(X[i, : self.nx], X[i, -self.nu :])
                        + self.model(X[i + 1, : self.nx], X[i + 1, -self.nu :])
                    )
                )

        ceq.extend((X[0, : self.nx] - x0).tolist())

        return ceq

    def ineq_constraints(
        self, X: npt.NDArray[np.float64], x_const: npt.NDArray[np.float64]
    ):
        X = X.reshape([self.n_pred, self.nx + self.nu], order="F")

        ineq = []
        x_robot = X[0, 0]
        y_robot = X[0, 1]

        x_obs = x_const[0]
        y_obs = x_const[1]

        ineq.append((x_robot - x_obs) ** 2 + (y_robot - y_obs) ** 2 - 30**2)

        return ineq

    def optimize(
        self,
        x0: npt.NDArray[np.float64],
        xref: npt.NDArray[np.float64],
        consts: npt.NDArray[np.float64],
    ):
        # dev_vars is used for warmstarting for the next step
        optimizer_dict = {
            "x": npt.NDArray[np.float64],
            "u": npt.NDArray[np.float64],
            "dec_vars": npt.NDArray[np.float64],
            "flag": False,
            "cost": float,
        }
        self.X = np.full([self.n_pred, self.nx + self.nu], np.nan)

        for i in range(self.nx + self.nu):
            # initial guess for states
            self.X[:, i] = np.linspace(0, self.x_guess, self.n_pred).flatten()

            if i > self.nx:
                self.X[:, i] = np.linspace(0.1, self.u_guess, self.n_pred).flatten()

        self.X = self.X.flatten(order="F")
        new_objective = lambda x: self.objective(x, xref)
        new_constraints = lambda x: self.constraints(x, x0)
        new_ineq_constraints = lambda x: self.ineq_constraints(x, consts)

        result = minimize(
            new_objective,
            self.X,
            method=self.minimize_method,
            constraints=[
                {"type": "eq", "fun": new_constraints},
                {"type": "ineq", "fun": new_ineq_constraints},
            ],
        )

        optimizer_dict["dec_vars"] = result["x"]

        res_reshaped = result.x.reshape([self.n_pred, self.nx + self.nu], order="F")
        optimizer_dict["x"] = res_reshaped[:, : self.nx]
        optimizer_dict["u"] = res_reshaped[:, -self.nu :]
        optimizer_dict["flag"] = result.success
        optimizer_dict["cost"] = result.fun

        if not result.success:
            print(f"Optimization failed with message '{result.message}' \nNow exiting.")
            sys.exit()

        return optimizer_dict
