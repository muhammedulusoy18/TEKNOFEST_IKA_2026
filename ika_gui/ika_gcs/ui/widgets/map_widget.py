from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtWebEngineWidgets import QWebEngineView


class MapWidget(QWidget):
    def __init__(self):
        super().__init__()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.view = QWebEngineView()
        layout.addWidget(self.view)

        map_html = """
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8" />
            <title>Harita</title>
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
            <link rel="stylesheet" 
                  href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
            <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
            <style>
                html, body {
                    margin: 0;
                    height: 100%;
                    background: #0f1724;
                    overflow: hidden;
                }
                #map {
                    width: 100%;
                    height: 100%;
                    border-radius: 10px;
                    border: 1px solid #1f2a3d;
                }
            </style>
        </head>
        <body>
            <div id="map"></div>
            <script>
                var map = L.map('map').setView([41.015137, 28.979530], 12);
                L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                    maxZoom: 19
                }).addTo(map);
                var marker = L.marker([41.015137, 28.979530]).addTo(map);

                function updateGps(lat, lng) {
                    marker.setLatLng([lat, lng]);
                    map.panTo([lat, lng]);
                }
            </script>
        </body>
        </html>
        """

        self.view.setHtml(map_html)

    def update_gps(self, t: dict):
        lat = t.get("lat") or t.get("latitude")
        lng = t.get("lng") or t.get("lon") or t.get("longitude")
        if lat is not None and lng is not None:
            self.view.page().runJavaScript(f"updateGps({lat}, {lng});")