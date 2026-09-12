#!/usr/bin/env python3
"""
Panel de control manual de parada/rearme para el Panda, con aspecto de
cuadro de mandos industrial (Tkinter). Complementa al boton fisico de la
Pico W (button_listener.py): publica directamente en /emergency_stop, asi
que sirve tanto para probar la parada sin tocar el hardware como para
tener un STOP a mano en el propio PC mientras corre una demo.

El boton de REARME empieza desactivado (en gris) -- solo se activa cuando
llega una parada de verdad (fisica o desde este mismo panel), igual que un
cuadro de mando real: no tiene sentido "rearmar" algo que no esta parado.
Al pulsarlo, publica /emergency_stop=False; cube_shuttle_demo.py y
stack_tower_demo.py (ver _wait_while_stopped) estaban BLOQUEADOS, no
abortados, asi que continuan exactamente desde el paso en el que se
quedaron, no desde el principio.

Nota: el topic no es "transient local", asi que si este panel arranca
DESPUES de una parada ya activa, no vera ese estado hasta el siguiente
mensaje -- arrancarlo junto con la demo, no a mitad de una parada.

Como lanzarlo (no depende de Webots ni del driver, en cualquier momento):

    ros2 run panda_controller estop_panel
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

import tkinter as tk
from tkinter import font as tkfont

BG = '#1c1c1c'
PANEL_BG = '#2a2a2a'
YELLOW = '#f5c400'
RED = '#c81e1e'
RED_DARK = '#8f1414'
GREEN = '#2e9e3b'
GREY = '#555555'
GREY_TEXT = '#aaaaaa'
TEXT_LIGHT = '#eaeaea'


class EstopPanelNode(Node):
    """Solo la parte ROS2 (publisher + subscripcion), sin Tkinter aqui --
    misma separacion que TeleopGuiNode en teleop_gui.py."""

    def __init__(self, on_state_change):
        super().__init__('estop_panel')
        self._on_state_change = on_state_change
        self.stopped = False
        self.publisher = self.create_publisher(Bool, '/emergency_stop', 10)
        self.pub_led = self.create_publisher(String, '/comando_led', 10)
        # La Pico del Loader tambien (sesion 2026-09-11): antes solo se avisaba
        # a la Pico W del Sorter y la del Loader seguia parpadeando tras el
        # rearme -- mismo criterio que teleop_gui.send_stop/send_rearm.
        self.pub_led_loader = self.create_publisher(String, '/comando_led_loader', 10)
        self.create_subscription(Bool, '/emergency_stop', self._on_emergency_stop, 10)

    def _on_emergency_stop(self, msg):
        if bool(msg.data) != self.stopped:
            self.stopped = bool(msg.data)
            self._on_state_change(self.stopped)

    def send_stop(self):
        self.publisher.publish(Bool(data=True))
        # Ademas del brazo (arriba), avisa tambien a la Pico W para que
        # parpadee en rojo -- mismo efecto visual que el boton fisico,
        # aunque la parada venga del panel (ver led_publisher.py /
        # Rasberry_Pi_Pico/main.py).
        self.pub_led.publish(String(data='parada'))
        self.pub_led_loader.publish(String(data='parada'))

    def send_rearm(self):
        self.publisher.publish(Bool(data=False))
        # Ademas del brazo (arriba), rearma tambien la Pico W: sin esto se
        # queda parpadeando en rojo para siempre, ya que main.py ignora a
        # proposito cualquier otro comando mientras dura la parada (ver
        # led_publisher.py / Rasberry_Pi_Pico/main.py).
        self.pub_led.publish(String(data='rearme'))
        self.pub_led_loader.publish(String(data='rearme'))


class EstopApp:
    def __init__(self, node: EstopPanelNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Panel de parada - Panda')
        self.root.configure(bg=BG)

        title_font = tkfont.Font(family='Arial', size=15, weight='bold')
        status_font = tkfont.Font(family='Arial', size=13, weight='bold')
        btn_font = tkfont.Font(family='Arial', size=13, weight='bold')

        header = tk.Label(self.root, text='CONTROL DE PARADA DE EMERGENCIA',
                           font=title_font, bg=BG, fg=YELLOW)
        header.grid(row=0, column=0, columnspan=2, padx=18, pady=(16, 4))

        # Tira de rayas amarillo/negro, puramente decorativa -- aspecto de
        # senalizacion industrial de zona de peligro/parada.
        stripe = tk.Canvas(self.root, width=360, height=10, bg=BG, highlightthickness=0)
        stripe.grid(row=1, column=0, columnspan=2, pady=(0, 12))
        n_stripes = 18
        w = 360 / n_stripes
        for i in range(n_stripes):
            color = YELLOW if i % 2 == 0 else '#000000'
            stripe.create_rectangle(i * w, 0, (i + 1) * w, 10, fill=color, outline='')

        panel = tk.Frame(self.root, bg=PANEL_BG, bd=4, relief='ridge')
        panel.grid(row=2, column=0, columnspan=2, padx=18, pady=(0, 8), sticky='nsew')

        self.status_var = tk.StringVar(value='EN MARCHA')
        self.status_label = tk.Label(panel, textvariable=self.status_var, font=status_font,
                                      bg=PANEL_BG, fg=GREEN, pady=10)
        self.status_label.grid(row=0, column=0, columnspan=2, sticky='we')

        self.stop_btn = tk.Button(
            panel, text='PARADA\nDE EMERGENCIA', font=btn_font, width=15, height=4,
            bg=RED, fg='white', activebackground=RED_DARK, activeforeground='white',
            relief='raised', bd=6, command=self.on_stop)
        self.stop_btn.grid(row=1, column=0, padx=14, pady=14)

        self.rearm_btn = tk.Button(
            panel, text='REARME', font=btn_font, width=15, height=4,
            relief='raised', bd=6, state='disabled', command=self.on_rearm)
        self.rearm_btn.grid(row=1, column=1, padx=14, pady=14)

        self.log_var = tk.StringVar(value='Listo.')
        log_label = tk.Label(self.root, textvariable=self.log_var, font=('monospace', 10),
                              bg=BG, fg=TEXT_LIGHT, wraplength=380, justify='left')
        log_label.grid(row=3, column=0, columnspan=2, padx=18, pady=(0, 16), sticky='w')

        self.set_stopped_ui(False)
        self._spin_tick()

    def on_stop(self):
        self.node.send_stop()
        self.log('Parada manual enviada desde el panel.')

    def on_rearm(self):
        self.node.send_rearm()
        self.log('Rearme enviado -- el brazo continua donde se quedo.')

    def log(self, msg):
        self.log_var.set(msg)

    def on_state_change(self, stopped):
        self.set_stopped_ui(stopped)
        if stopped:
            self.log('PARADA DE EMERGENCIA ACTIVA (fisica o manual).')

    def set_stopped_ui(self, stopped):
        if stopped:
            self.status_var.set('PARADO')
            self.status_label.config(fg=RED)
            self.rearm_btn.config(state='normal', bg=YELLOW, fg='black',
                                   activebackground='#c9a400', activeforeground='black')
        else:
            self.status_var.set('EN MARCHA')
            self.status_label.config(fg=GREEN)
            self.rearm_btn.config(state='disabled', bg=GREY, fg=GREY_TEXT,
                                   activebackground=GREY, activeforeground=GREY_TEXT)

    def _spin_tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(50, self._spin_tick)

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    app_holder = {}

    def on_state_change(stopped):
        app_holder['app'].on_state_change(stopped)

    node = EstopPanelNode(on_state_change)
    app = EstopApp(node)
    app_holder['app'] = app
    try:
        app.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
