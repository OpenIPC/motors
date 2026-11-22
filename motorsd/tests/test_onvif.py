#!/usr/bin/env python3
"""Host integration test for motors-onvif against motorsd + mock_driver."""
import base64
import hashlib
import os
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from datetime import datetime, timezone

BUILD_DIR = os.path.join(os.path.dirname(__file__), "..", "build")


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def soap_call(port, path, body, user="onvif", pwd="secret", auth=True):
    hdr = ""
    if auth:
        nonce = os.urandom(16)
        created = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        digest = base64.b64encode(
            hashlib.sha1(nonce + created.encode() + pwd.encode()).digest()
        ).decode()
        nonce_b64 = base64.b64encode(nonce).decode()
        hdr = (
            '<s:Header><wsse:Security '
            'xmlns:wsse="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd" '
            'xmlns:wsu="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd">'
            f"<wsse:UsernameToken><wsse:Username>{user}</wsse:Username>"
            '<wsse:Password Type="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordDigest">'
            f"{digest}</wsse:Password>"
            '<wsse:Nonce EncodingType="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-soap-message-security-1.0#Base64Binary">'
            f"{nonce_b64}</wsse:Nonce><wsu:Created>{created}</wsu:Created>"
            "</wsse:UsernameToken></wsse:Security></s:Header>"
        )
    env = (
        f'<s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope">'
        f"{hdr}<s:Body>{body}</s:Body></s:Envelope>"
    )
    req = urllib.request.Request(
        f"http://127.0.0.1:{port}{path}",
        data=env.encode(),
        headers={"Content-Type": "application/soap+xml; charset=utf-8"},
    )
    with urllib.request.urlopen(req, timeout=5) as r:
        txt = r.read().decode()
        ET.fromstring(txt)
        return r.status, txt


def main():
    with tempfile.TemporaryDirectory() as tmp:
        sock = os.path.join(tmp, "motorsd.sock")
        trace = os.path.join(tmp, "trace.log")
        drv_cfg = os.path.join(tmp, "mock.conf")
        with open(drv_cfg, "w") as f:
            f.write(f"trace={trace}\n")
        presets = os.path.join(tmp, "presets.tsv")
        with open(presets, "w") as f:
            f.write("1|Home|1.0|0|1|1000|500|0\n2|Driveway|12.0|0|1|5000|800|0\n")

        port = free_port()
        md = subprocess.Popen(
            [
                os.path.join(BUILD_DIR, "motorsd"),
                "--socket", sock,
                "--driver", os.path.join(BUILD_DIR, "mock_driver"),
                "--driver-config", drv_cfg,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        onvif = None
        try:
            for _ in range(30):
                if os.path.exists(sock):
                    break
                time.sleep(0.05)
            assert os.path.exists(sock), "motorsd socket did not appear"

            onvif = subprocess.Popen(
                [
                    os.path.join(BUILD_DIR, "motors-onvif"),
                    "--port", str(port),
                    "--socket", sock,
                    "--presets", presets,
                    "--user", "onvif",
                    "--pass", "secret",
                    "--no-post-zoom-af",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            time.sleep(0.2)

            # Unauthenticated PTZ call must fail with 401
            try:
                soap_call(
                    port,
                    "/onvif/ptz_service",
                    '<tptz:GetPresets xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl"><tptz:ProfileToken>PROFILE_000</tptz:ProfileToken></tptz:GetPresets>',
                    auth=False,
                )
                raise AssertionError("unauthenticated GetPresets should fail")
            except urllib.error.HTTPError as e:
                assert e.code == 401, f"expected 401 got {e.code}"

            # Capabilities & Services
            st, xml = soap_call(
                port, "/onvif/device_service",
                '<tds:GetCapabilities xmlns:tds="http://www.onvif.org/ver10/device/wsdl"><tds:Category>All</tds:Category></tds:GetCapabilities>',
                auth=False,
            )
            assert st == 200 and "/onvif/ptz_service" in xml and "/onvif/image_service" in xml

            # Configuration options (FOV + Zoom)
            st, xml = soap_call(
                port, "/onvif/ptz_service",
                '<tptz:GetConfigurationOptions xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl"><tptz:ConfigurationToken>PTZ_CFG_000</tptz:ConfigurationToken></tptz:GetConfigurationOptions>',
            )
            assert st == 200 and "TranslationSpaceFov" in xml and "RelativeZoomTranslationSpace" in xml

            # Presets
            st, xml = soap_call(
                port, "/onvif/ptz_service",
                '<tptz:GetPresets xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl"><tptz:ProfileToken>PROFILE_000</tptz:ProfileToken></tptz:GetPresets>',
            )
            assert st == 200 and "Home" in xml and "Driveway" in xml

            # RelativeMove + Stop
            st, _ = soap_call(
                port, "/onvif/ptz_service",
                '<tptz:RelativeMove xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl" xmlns:tt="http://www.onvif.org/ver10/schema"><tptz:ProfileToken>PROFILE_000</tptz:ProfileToken><tptz:Translation><tt:PanTilt x="0.1" y="0"/></tptz:Translation></tptz:RelativeMove>',
            )
            assert st == 200
            st, _ = soap_call(
                port, "/onvif/ptz_service",
                '<tptz:Stop xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl"><tptz:ProfileToken>PROFILE_000</tptz:ProfileToken><tptz:PanTilt>true</tptz:PanTilt><tptz:Zoom>true</tptz:Zoom></tptz:Stop>',
            )
            assert st == 200

            # Imaging GetMoveOptions & GetStatus
            st, xml = soap_call(
                port, "/onvif/image_service",
                '<timg:GetMoveOptions xmlns:timg="http://www.onvif.org/ver20/imaging/wsdl"><timg:VideoSourceToken>VS_000</timg:VideoSourceToken></timg:GetMoveOptions>',
            )
            assert st == 200 and "Continuous" in xml
            st, xml = soap_call(
                port, "/onvif/image_service",
                '<timg:GetStatus xmlns:timg="http://www.onvif.org/ver20/imaging/wsdl"><timg:VideoSourceToken>VS_000</timg:VideoSourceToken></timg:GetStatus>',
            )
            assert st == 200 and "FocusStatus20" in xml

            print("test_onvif.py: ALL PASSED")
        finally:
            if onvif:
                onvif.terminate()
                onvif.wait(timeout=3)
            md.terminate()
            md.wait(timeout=3)


if __name__ == "__main__":
    sys.exit(main())
