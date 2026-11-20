"""Dugumleri Ctrl-C'de ARACI GERCEKTEN DURDURARAK kosan ortak kosucu.

Neden ayri bir dosya: uc dugumun de ayni kapanis problemi var ve problem
sessiz. Ilk Gazebo kosusunda olculdu.

PROBLEM
-------
Alisilmis kalip sudur:

    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.halt()          # <-- araci durdurmasi beklenir
        node.destroy_node()
        rclpy.shutdown()

Ctrl-C'ye basildiginda bu kalip CALISMAZ. `rclpy.init` varsayilan olarak
KENDI SIGINT isleyicisini kurar; sinyal gelince once rclpy context'i
kapatilir, sonra kontrol `finally` blogumuza doner. O noktada yayinci artik
gecersizdir:

    rcl publisher's context is invalid, at ./src/rcl/publisher.c:389

Sonuc iki katli: (a) ekrana cirkin bir traceback dusuyor, (b) daha onemlisi
ARACI DURDURMA KOMUTU HIC GITMIYOR. Gazebo'nun diff_drive eklentisi son
`cmd_vel` degerini korur; yani yalnizca dugumu oldurursen robot son komutla
yoluna devam eder.

COZUM
-----
Sinyal isleyicisini rclpy'ye birakmiyoruz (`SignalHandlerOptions.NO`), kendi
isleyicimizi kuruyoruz. Boylece SIGINT geldiginde context HALA GECERLIyken
once durdurma komutunu yayinlayip sonra duzgunce cikiyoruz.

GERCEK ARACTA
-------------
Orada ek bir guvenlik katmani var: `/beemobs/AutonomousHeardBit` canlilik
sinyali kesilirse arac otonom moddan kendiliginden cikar. Yani dugum
cokerse arac durur. Simulasyonda boyle bir mekanizma olmadigi icin
durdurmayi bizim garanti etmemiz gerekiyor.
"""
from __future__ import annotations

import signal
from typing import Callable, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

# Bir dugumun "dur" yordaminin birden fazla kez cagrilmasi zararsiz olmali;
# yine de cift cagriyi burada da engelliyoruz.


def run(
    node_factory: Callable[[], Node],
    on_stop: str = "halt",
    args: Optional[list] = None,
) -> None:
    """Dugumu kosar; SIGINT/SIGTERM'de `on_stop` yordamini CONTEXT GECERLIYKEN cagirir.

    `node_factory`: dugumu ureten cagrilabilir (kurucu hatalari yakalanabilsin
    diye dugumun kendisi degil, ureticisi aliniyor).
    `on_stop`: kapanista cagrilacak yordamin adi ("halt", "finish", ...).
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node: Optional[Node] = None
    already_stopped = False

    def stop(reason: str) -> None:
        nonlocal already_stopped
        if already_stopped or node is None:
            return
        already_stopped = True
        hook = getattr(node, on_stop, None)
        if hook is None:
            return
        try:
            hook()
            node.get_logger().info(f"kapanis: {on_stop}() calisti ({reason})")
        except Exception as exc:                      # noqa: BLE001
            # Kapanista istisna firlatmak, geri kalan temizligi engeller.
            node.get_logger().error(f"kapanista {on_stop}() hata verdi: {exc}")

    def handle_signal(signum, _frame) -> None:
        # SONRAKI sinyalleri YOKSAY.
        #
        # Neden: Ctrl-C genelde BIR KEZDEN FAZLA gelir -- `timeout --signal=INT`
        # surec grubuna yollar, `ros2 launch` da kendi cocuklarina yollar.
        # Ikinci sinyal, biz `finally` blogunda temizlik yaparken gelir ve
        # `destroy_node()`'u yarida keser. Olculdu: traceback tam
        # `destroy_subscription` icinde patladi. Ilk sinyali isliyoruz,
        # gerisini gormezden geliyoruz.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)

        # Burada context HALA GECERLI -- durdurma komutu gercekten gidiyor.
        stop(signal.Signals(signum).name)
        raise KeyboardInterrupt

    try:
        node = node_factory()
        signal.signal(signal.SIGINT, handle_signal)
        signal.signal(signal.SIGTERM, handle_signal)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # Baska bir yerden shutdown cagrildi; context gitmis olabilir.
        pass
    finally:
        stop("normal cikis")
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
