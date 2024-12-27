import axios from 'axios';
import * as dotenv from 'dotenv';
dotenv.config();

export async function createLog(
  user: string,
  action: string,
  category: string,
  local: string,
  status: string,
  user_id?: number,
): Promise<void> {

  try {
    const apiUrl = 'http://localhost:3000/logs';

    console.log('URL da API:', apiUrl);

    const logData = {
      user,
      action,
      category,
      local,
      status,
      user_id,
    };

    await axios.post(apiUrl, logData);

    console.log('Log criado com sucesso:', logData);
  } catch (error) {
    console.error('Erro ao criar log:', error.message);
  }
}
