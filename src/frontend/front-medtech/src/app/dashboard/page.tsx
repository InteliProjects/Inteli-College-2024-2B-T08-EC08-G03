"use client";

import React, { useEffect, useState } from "react";
import Navbar from "../components/Navbar"; 
import { Bar, Line } from "react-chartjs-2";
import { Button } from "@/components/ui/button";
import { RefreshCcw } from "lucide-react";
import { Battery } from "lucide-react";
import {
  Select,
  SelectContent,
  SelectGroup,
  SelectItem,
  SelectLabel,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select"
import "chart.js/auto";
import { requisicoesDataDia,
  requisicoesDataSemana,
  requisicoesDataMes,
  requisicoesDataSemestre,
  requisicoesDataAno,
  mediaTempoData,
  mediaTempoManha,
  mediaTempoNoite,
  pedidosRealizadosManha,
  pedidosRealizadosNoite,
  pedidosRealizadosHoje,
  pedidosCanceladosManha,
  pedidosCanceladosNoite,
  pedidosCanceladosHoje,
  robotsLocation

 } from "./mock-data";

export default function Dashboard() {
  const [isClient, setIsClient] = useState(false);
  const dateOptions = {
    "Hoje": requisicoesDataDia,
    "Essa semana": requisicoesDataSemana,
    "Esse mês": requisicoesDataMes,
    "Esse semestre": requisicoesDataSemestre,
    "Esse ano": requisicoesDataAno
  }

  const timeFinishedOptions = {
    "00h-12h": pedidosRealizadosManha,
    "12h-00h": pedidosRealizadosNoite,
    "Hoje": pedidosRealizadosHoje,
  }

  const timeCanceledOptions = {
    "00h-12h": pedidosCanceladosManha,
    "12h-00h": pedidosCanceladosNoite,
    "Hoje": pedidosCanceladosHoje,
  }


  const timeOptions = {
    "00h-12h": mediaTempoManha,
    "12h-00h": mediaTempoNoite,
    "Hoje": mediaTempoData,
  }
  const [selectedCategoryData, setSelectedCategoryData] = useState(requisicoesDataDia);
  const [selectedTimeData, setSelectedTimeData] = useState(mediaTempoManha);
  const [selectedFinishedData, setSelectedFinishedData] = useState(pedidosRealizadosHoje);
  const [selectedCanceledData, setSelectedCanceledData] = useState(pedidosCanceladosHoje);

  useEffect(() => {
    setIsClient(true);
  }, []);

  

  if (!isClient) {
    return null;
  }

  return (
    <>
      <Navbar />
      <div className="p-5">
        <div className="grid grid-cols-2 gap-5">
          {/* Gráfico 1: Quantidade de requisições */}
          <div className="bg-white rounded-lg shadow-md p-5 text-center">
            <h2 className="text-xl font-semibold mb-3">
              Quantidade de requisições hoje por tipo de requisição
            </h2>
            <Select 
            defaultValue="Hoje"
            onValueChange={(value) => {setSelectedCategoryData(Object.values(dateOptions[value]))
            }}>
              <SelectTrigger>
                <SelectValue placeholder="Selecione o período" />
              </SelectTrigger>
              <SelectContent>
                <SelectGroup>
                  <SelectLabel>Período</SelectLabel>
                  {Object.keys(dateOptions).map((key) => (
                    <SelectItem key={key} value={key}>
                      {key}
                      </SelectItem>
                  ))}
                </SelectGroup>
              </SelectContent>
            </Select>
            <Bar data={{
              labels: ["Alimentação", "Roupa de cama", "Insumos médicos"],
              datasets: [
                {
                  label: "Quantidade de requisições",
                  data: [...Object.values(selectedCategoryData) as number[]],
                  borderColor: "#42A5F5",
                  backgroundColor: ["#E57373", "#64B5F6", "#81C784"],
                },
              ]}} />
          </div>

          {/* Gráfico 2: Média de tempo */}
          <div className="bg-white rounded-lg shadow-md p-5 text-center">
            <h2 className="text-xl font-semibold mb-3">
              Média de tempo das requisições ao longo do dia
            </h2>
            <Select 
            defaultValue="Hoje"
            onValueChange={(value) => {setSelectedTimeData(Object.values(timeOptions[value]))
            }}>
              <SelectTrigger>
                <SelectValue placeholder="Selecione o período" />
              </SelectTrigger>
              <SelectContent>
                <SelectGroup>
                  <SelectLabel>Período</SelectLabel>
                  {Object.keys(timeOptions).map((key) => (
                    <SelectItem key={key} value={key}>
                      {key}
                      </SelectItem>
                  ))}
                </SelectGroup>
              </SelectContent>
            </Select>
            <Line data={{
              labels: [...Object.keys(selectedTimeData)],
              datasets: [
                {
                  label: "Tempo médio (min)",
                  data: [...Object.values(selectedTimeData)],
                  borderColor: "#424242",
                  backgroundColor: "rgba(66, 66, 66, 0.5)",
                  tension: 0.4,
                },
              ],
            }} />
          </div>

          {/* Gráfico 3: Pedidos realizados x cancelados */}
          <div className="bg-white rounded-lg shadow-md p-5 text-center">
            <h2 className="text-l font-semibold mb-3">
              Pedidos realizados ao longo do dia x Pedidos cancelados ao longo
              do dia
            </h2>
            <Select 
            defaultValue="Hoje"
            onValueChange={(value) => {setSelectedFinishedData(Object.values(timeFinishedOptions[value]));
              setSelectedCanceledData(Object.values(timeCanceledOptions[value]))
            }}>
              <SelectTrigger>
                <SelectValue placeholder="Selecione o período" />
              </SelectTrigger>
              <SelectContent>
                <SelectGroup>
                  <SelectLabel>Período</SelectLabel>
                  {Object.keys(timeFinishedOptions).map((key) => (
                    <SelectItem key={key} value={key}>
                      {key}
                      </SelectItem>
                  ))}
                </SelectGroup>
              </SelectContent>
            </Select>
            <Line data={{
              labels: [...Object.keys(selectedFinishedData)],
              datasets: [
                {
                  label: "Pedidos realizados",
                  data: [...Object.values(selectedFinishedData)],
                  borderColor: "#424242",
                  backgroundColor: "rgba(66, 66, 66, 0.5)",
                  tension: 0.4,
                },
                {
                  label: "Pedidos cancelados",
                  data: [...Object.values(selectedCanceledData)],
                  borderColor: "#E57373",
                  backgroundColor: "rgba(229, 115, 115, 0.5)",
                  tension: 0.4,
                },
              ],
            }} />
          </div>

          {/* Mapa 1 */}
          <div className="bg-white rounded-lg shadow-md p-5 text-center">
            <div className="flex flex-row text-center justify-center	">
            <h2 className="text-xl font-semibold mb-3 px-3">
              Funcionamento dos robôs
            </h2>
            <Button
              onClick={()=> window.location.reload()}
              variant="outline"
            >
              <RefreshCcw size={20} />
            </Button>
            </div>
                <div className="space-y-2">
                {Object.keys(robotsLocation).map((key) => {
                  const value = robotsLocation[key];
                  let bgColor = "bg-red-500";
                  if (value > 50) {
                  bgColor = "bg-green-500";
                  } else if (value > 20) {
                  bgColor = "bg-yellow-500";
                  }

                  return (
                    <div key={key} className="flex justify-between items-center">
                    <span className="px-2 text-center font-semibold">{key}</span>
                    <span className="p-1 text-sm">{`${value}%`}</span>
                    <div className="w-3/4 bg-gray-300 rounded h-4">
                      <div
                      className={`h-4 rounded ${bgColor}`}
                      style={{ width: `${value}%` }}
                      ></div>
                    </div>
                    </div>
                  );
                })}
                </div>
            </div>
        </div>
      </div>
    </>
  );
}