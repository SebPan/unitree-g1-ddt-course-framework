import time


FSM_ZERO = 0
FSM_DAMP = 1
FSM_STAND = 4
FSM_WALK = 500


class G1Locomotion:

    def __init__(self, motion_switcher, loco):
        self.msc = motion_switcher
        self.loco = loco

    # ---------------------------------------------------------
    # ESTADO
    # ---------------------------------------------------------

    def get_fsm(self):
        try:
            return self.loco.GetFsmId()
        except Exception as e:
            print("[LOCO] GetFsmId error:", e)
            return -1, None

    def wait_fsm(self, expected, timeout=5.0):
        t0 = time.time()

        while time.time() - t0 < timeout:
            code, fsm = self.get_fsm()

            if code == 0 and fsm == expected:
                print(f"[LOCO] FSM confirmado: {fsm}")
                return True

            time.sleep(0.1)

        code, fsm = self.get_fsm()

        print(
            f"[LOCO] Timeout esperando FSM {expected}. "
            f"Actual={fsm}, code={code}"
        )

        return False

    # ---------------------------------------------------------
    # MOTION SWITCHER
    # ---------------------------------------------------------

    def ensure_high_level(self):
        status, mode = self.msc.CheckMode()

        name = None

        if status == 0 and mode:
            name = mode.get("name")

        print("[LOCO] MotionSwitcher:", name)

        if name:
            return True

        print("[LOCO] Activando modo ai...")

        ret = self.msc.SelectMode("ai")

        if ret != 0:
            print("[LOCO] SelectMode('ai') fallo:", ret)
            return False

        time.sleep(2.0)

        status, mode = self.msc.CheckMode()

        name = mode.get("name") if mode else None

        print("[LOCO] MotionSwitcher ahora:", name)

        return status == 0 and bool(name)

    # ---------------------------------------------------------
    # FSM
    # ---------------------------------------------------------

    def set_fsm(self, target, timeout=5.0):
        print(f"[LOCO] SetFsmId({target})")

        ret = self.loco.SetFsmId(target)

        print("[LOCO] ret =", ret)

        if ret != 0:
            return False

        return self.wait_fsm(target, timeout)

    # ---------------------------------------------------------
    # API
    # ---------------------------------------------------------

    def status(self):
        code, fsm = self.get_fsm()

        status, mode = self.msc.CheckMode()
        mode_name = mode.get("name") if mode else None

        return {
            "ok": code == 0,
            "fsm": fsm,
            "motion_mode": mode_name,
        }

    def damp(self):
        print("[LOCO] DAMP")

        try:
            self.loco.SetVelocity(
                0.0, 0.0, 0.0, 1.0
            )
        except Exception:
            pass

        return self.set_fsm(FSM_DAMP)

    def stand(self):
        print("[LOCO] STAND")

        code, fsm = self.get_fsm()

        print(f"[LOCO] FSM antes de stand: {fsm}")

        if code != 0:
            return False

        if fsm == FSM_STAND:
            print("[LOCO] Ya esta en STAND")
            return True

        # Si venimos de WALKING, primero detenemos
        # y regresamos por DAMP.
        if fsm == FSM_WALK:
            print("[LOCO] WALK -> STOP -> DAMP -> STAND")

            self.stop()
            time.sleep(0.5)

            if not self.set_fsm(FSM_DAMP):
                return False

            time.sleep(1.0)

        # Si estamos en ZERO TORQUE, primero DAMP.
        elif fsm == FSM_ZERO:
            print("[LOCO] ZERO -> DAMP -> STAND")

            if not self.set_fsm(FSM_DAMP):
                return False

            time.sleep(1.0)

        # Cualquier otro estado diferente de DAMP
        # lo llevamos primero a DAMP.
        elif fsm != FSM_DAMP:
            print(
                f"[LOCO] FSM {fsm} -> DAMP -> STAND"
            )

            if not self.set_fsm(FSM_DAMP):
                return False

            time.sleep(1.0)

        return self.set_fsm(FSM_STAND)

    def prepare_walk(self):
        code, fsm = self.get_fsm()

        if code != 0:
            return False

        if fsm == FSM_WALK:
            return True

        if fsm != FSM_STAND:
            if not self.stand():
                return False

            time.sleep(1.0)

        return self.set_fsm(FSM_WALK)

    def velocity(
        self,
        vx,
        vy=0.0,
        wz=0.0,
        duration=1.0
    ):
        if not self.prepare_walk():
            return False

        print(
            f"[LOCO] velocity "
            f"vx={vx:.3f} "
            f"vy={vy:.3f} "
            f"wz={wz:.3f}"
        )

        ret = self.loco.SetVelocity(
            float(vx),
            float(vy),
            float(wz),
            float(duration),
        )

        return ret == 0

    def stop(self):
        print("[LOCO] STOP")

        ret = self.loco.SetVelocity(
            0.0,
            0.0,
            0.0,
            1.0,
        )

        return ret == 0

    def zero_torque(self):
        self.stop()
        time.sleep(0.2)

        return self.set_fsm(FSM_ZERO)
