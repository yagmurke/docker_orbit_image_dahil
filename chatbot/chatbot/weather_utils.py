import os
import requests
import json
from datetime import datetime, timedelta
import pytz
import re
from typing import Tuple, Optional, Dict

# OpenWeatherMap API yapılandırması
API_KEY = "YOUR_OPENWEATHERMAP_API_KEY"  # Kendi API anahtarınızla değiştirin
BASE_URL = "http://api.openweathermap.org/data/2.5/weather"
FORECAST_URL = "http://api.openweathermap.org/data/2.5/forecast"
GEOCODING_URL = "http://api.openweathermap.org/geo/1.0/direct"

# Türkçe hava durumu açıklamaları
WEATHER_DESCRIPTIONS = {
    "Clear": "açık",
    "Clouds": "bulutlu",
    "Rain": "yağmurlu",
    "Drizzle": "çiseleyen",
    "Thunderstorm": "gök gürültülü fırtına",
    "Snow": "karlı",
    "Mist": "puslu",
    "Smoke": "dumanlı",
    "Haze": "sisli",
    "Dust": "tozlu",
    "Fog": "yoğun sisli",
    "Sand": "kumlu",
    "Ash": "küllü",
    "Squall": "fırtınalı",
    "Tornado": "kasırgalı"
}

# Yönlendirme açıklamaları
WIND_DIRECTIONS = {
    "N": "kuzeyden",
    "NE": "kuzeydoğudan",
    "E": "doğudan",
    "SE": "güneydoğudan",
    "S": "güneyden",
    "SW": "güneybatıdan",
    "W": "batıdan",
    "NW": "kuzeybatıdan"
}

def get_weather_data(city: str, country: str = "TR") -> Optional[Dict]:
    """
    Belirtilen şehir için güncel hava durumu verilerini al
    """
    try:
        # Önce şehrin koordinatlarını bul
        geo_params = {
            "q": f"{city},{country}",
            "limit": 1,
            "appid": API_KEY
        }
        
        geo_response = requests.get(GEOCODING_URL, params=geo_params)
        geo_response.raise_for_status()
        
        locations = geo_response.json()
        if not locations:
            return None
            
        lat = locations[0]["lat"]
        lon = locations[0]["lon"]
        
        # Hava durumu verilerini al
        weather_params = {
            "lat": lat,
            "lon": lon,
            "appid": API_KEY,
            "units": "metric",
            "lang": "tr"
        }
        
        # Anlık hava durumu
        current_response = requests.get(BASE_URL, params=weather_params)
        current_response.raise_for_status()
        current_data = current_response.json()
        
        # 5 günlük tahmin
        forecast_response = requests.get(FORECAST_URL, params=weather_params)
        forecast_response.raise_for_status()
        forecast_data = forecast_response.json()
        
        return {
            "current": current_data,
            "forecast": forecast_data
        }
        
    except requests.exceptions.RequestException as e:
        print(f"Hava durumu verileri alınamadı: {str(e)}")
        return None

def get_wind_direction(degrees: float) -> str:
    """
    Rüzgar yönünü dereceye göre Türkçe olarak döndür
    """
    directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    index = round(degrees / (360 / len(directions))) % len(directions)
    return WIND_DIRECTIONS[directions[index]]

def extract_location(query: str) -> Tuple[str, str]:
    """
    Metin içinden şehir ve ülke bilgisini çıkart
    """
    # Varsayılan değerler
    city = "Istanbul"  # Varsayılan şehir
    country = "TR"    # Varsayılan ülke
    
    # Yaygın şehir isimleri için regex kalıpları
    city_patterns = [
        r'(?i)(istanbul|ankara|izmir|bursa|antalya|adana|konya|mersin|diyarbakır|eskişehir)\'(?:da|de|ta|te)',
        r'(?i)(istanbul|ankara|izmir|bursa|antalya|adana|konya|mersin|diyarbakır|eskişehir)(?:\s|$)',
    ]
    
    # Metinde şehir ismi ara
    for pattern in city_patterns:
        match = re.search(pattern, query)
        if match:
            city = match.group(1).title()
            break
    
    return city, country

def get_weather_reply(query: str) -> str:
    """
    Hava durumu sorgusuna Türkçe yanıt oluştur
    """
    try:
        # Konumdan şehir bilgisini çıkart
        city, country = extract_location(query)
        
        # Hava durumu verilerini al
        weather_data = get_weather_data(city, country)
        if not weather_data:
            return f"{city} için hava durumu bilgisi alınamadı."
        
        current = weather_data["current"]
        forecast = weather_data["forecast"]
        
        # Mevcut hava durumu bilgileri
        temp = round(current["main"]["temp"])
        feels_like = round(current["main"]["feels_like"])
        humidity = current["main"]["humidity"]
        wind_speed = round(current["wind"]["speed"] * 3.6)  # m/s'den km/h'ye çevir
        wind_direction = get_wind_direction(current["wind"]["deg"])
        description = WEATHER_DESCRIPTIONS.get(
            current["weather"][0]["main"],
            current["weather"][0]["description"]
        )
        
        # Zaman kontrolü
        now = datetime.now(pytz.timezone('Europe/Istanbul'))
        is_evening = now.hour >= 18 or now.hour < 6
        
        # Yanıt oluştur
        response = f"{city}'da hava {description}. "
        response += f"Sıcaklık {temp}°C, hissedilen {feels_like}°C. "
        
        # Rüzgar bilgisi
        if wind_speed > 0:
            response += f"Rüzgar {wind_direction} {wind_speed} km/h hızla esiyor. "
        
        # Nem bilgisi
        if humidity > 70:
            response += f"Nem oranı yüksek (%{humidity}). "
        elif humidity < 30:
            response += f"Hava oldukça kuru (%{humidity} nem). "
        
        # Yakın zaman tahmini
        next_hours = forecast["list"][:3]
        rain_probability = any(
            hour["weather"][0]["main"] in ["Rain", "Thunderstorm"]
            for hour in next_hours
        )
        
        if rain_probability:
            response += "Önümüzdeki saatlerde yağış bekleniyor. "
        elif is_evening:
            response += "Gece boyunca önemli bir yağış beklenmiyor. "
        else:
            response += "Gün içinde önemli bir yağış beklenmiyor. "
        
        return response.strip()
        
    except Exception as e:
        print(f"Hava durumu yanıtı oluşturulurken hata: {str(e)}")
        return "Üzgünüm, hava durumu bilgisi alınamadı."