FROM node:18 AS build
WORKDIR /app
COPY package*.json ./
RUN npm install && npm install -g @nestjs/cli
COPY . .
RUN npm run build

FROM node:18-alpine AS production
WORKDIR /app
COPY package*.json ./
RUN npm install && npm ci --only=production && npm install -g @nestjs/cli
COPY . .
EXPOSE 3000
CMD ["node", "dist/main"]
